package dev.superyojan.terra.phone

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.os.Handler
import android.os.SystemClock
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.UnavailableException
import dev.superyojan.terra.core.ImuReading
import dev.superyojan.terra.core.MappingDepthFrame
import dev.superyojan.terra.core.PhonePose
import dev.superyojan.terra.core.PhonePosePipeline
import dev.superyojan.terra.core.arTimeOnSensorClock
import dev.superyojan.terra.core.defaultMinDepthConfidence
import dev.superyojan.terra.core.depth16ToMetres
import dev.superyojan.terra.core.latchGroundOffset
import dev.superyojan.terra.core.yUpCameraToOpticalPose
import java.nio.ByteBuffer
import kotlin.math.abs

/**
 * Platform sensor shell. The mount, offset, velocity filter, depth decoding
 * and ground latch are Rust calls. This class only reads SensorManager and ARCore.
 *
 * Call [start], [poll], and [close] on the sensor thread.
 */
class PhoneSensors(
    context: Context,
    private val handler: Handler,
    private val pipeline: PhonePosePipeline,
    /** Camera height above the floor from a plausible mount calibration, metres. */
    private val mountHeight: () -> Double?,
    private val onImu: (ImuReading) -> Unit,
    private val onPose: (PhonePose) -> Unit,
    private val onDepth: (MappingDepthFrame) -> Unit,
) : SensorEventListener {
    private val appContext = context.applicationContext
    private val sensors = appContext.getSystemService(SensorManager::class.java)
    private var session: Session? = null
    private var egl = HeadlessEgl()
    private var depthSupported = false
    private var groundOffset: Double? = null
    private var lastDepthTime = Double.NEGATIVE_INFINITY
    private var lastFrameNanos = Long.MIN_VALUE
    private var accel: FloatArray? = null
    private var gyro: FloatArray? = null
    private var accelTime = 0.0
    private var gyroTime = 0.0
    var status = "Phone tracking is unavailable; use Simulated rover."
        private set

    fun start(): Boolean {
        val linear = sensors?.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        val gyroscope = sensors?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (sensors == null || linear == null || gyroscope == null) {
            status = "Phone tracking is unavailable; use Simulated rover."
            return false
        }
        val ar = try {
            Session(appContext)
        } catch (_: UnavailableException) {
            status = "Phone tracking is unavailable; use Simulated rover."
            return false
        }
        if (!egl.start()) {
            ar.close()
            status = "Phone tracking is unavailable; use Simulated rover."
            return false
        }
        val config = Config(ar).apply {
            updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            planeFindingMode = Config.PlaneFindingMode.DISABLED
            lightEstimationMode = Config.LightEstimationMode.DISABLED
            // Automatic depth also produces the raw depth and confidence images.
            if (ar.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
                depthMode = Config.DepthMode.AUTOMATIC
                depthSupported = true
            } else {
                depthMode = Config.DepthMode.DISABLED
                depthSupported = false
            }
        }
        return try {
            ar.configure(config)
            ar.setCameraTextureName(egl.textureId)
            val metrics = appContext.resources.displayMetrics
            ar.setDisplayGeometry(0, metrics.widthPixels, metrics.heightPixels)
            pipeline.restart(monotonicSeconds())
            ar.resume()
            session = ar
            sensors.registerListener(this, linear, SensorManager.SENSOR_DELAY_GAME, handler)
            sensors.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME, handler)
            status = if (depthSupported) {
                "Waiting for scene depth"
            } else {
                "Scene depth unavailable on this device"
            }
            true
        } catch (_: UnavailableException) {
            ar.close()
            egl.close()
            status = "Phone tracking is unavailable; use Simulated rover."
            false
        }
    }

    fun depthMessage(): String = status

    fun poll() {
        val ar = session ?: return
        val frame = try {
            ar.update()
        } catch (_: Exception) {
            return
        }
        // LATEST_CAMERA_IMAGE hands back the same frame until a new image arrives.
        if (frame.timestamp == lastFrameNanos) return
        lastFrameNanos = frame.timestamp
        val camera = frame.camera
        val tracked = camera.trackingState == TrackingState.TRACKING
        val timestamp = arTimeOnSensorClock(
            frame.timestamp.toDouble() / 1_000_000_000.0,
            System.nanoTime().toDouble() / 1_000_000_000.0,
            monotonicSeconds(),
        )
        // Device axes (+X right, +Y top edge, +Z out of the screen), whatever the display rotation.
        val device = frame.androidSensorPose
        val position = device.translation
        val rotation = device.rotationQuaternion
        val pose = try {
            pipeline.pushPose(
                timestamp,
                tracked,
                position[0].toDouble(),
                position[1].toDouble(),
                position[2].toDouble(),
                rotation[0].toDouble(),
                rotation[1].toDouble(),
                rotation[2].toDouble(),
                rotation[3].toDouble(),
            )
        } catch (_: Exception) {
            // A malformed tracked pose is dropped; the controller's freshness check stops motion.
            null
        } ?: return
        onPose(pose)
        if (tracked && depthSupported && timestamp - lastDepthTime >= 0.1) {
            val optics = camera.pose
            depthFrame(frame, optics.translation, optics.rotationQuaternion, timestamp)?.let { depth ->
                lastDepthTime = timestamp
                onDepth(depth)
            }
        }
    }

    fun close() {
        sensors?.unregisterListener(this)
        session?.pause()
        session?.close()
        session = null
        egl.close()
        groundOffset = null
        lastFrameNanos = Long.MIN_VALUE
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_LINEAR_ACCELERATION -> {
                accel = event.values.clone()
                accelTime = event.timestamp.toDouble() / 1_000_000_000.0
            }
            Sensor.TYPE_GYROSCOPE -> {
                gyro = event.values.clone()
                gyroTime = event.timestamp.toDouble() / 1_000_000_000.0
            }
            else -> return
        }
        val acceleration = accel ?: return
        val rate = gyro ?: return
        if (abs(accelTime - gyroTime) > 0.05) return
        val reading = try {
            pipeline.imu(
                maxOf(accelTime, gyroTime),
                acceleration[0].toDouble(),
                acceleration[1].toDouble(),
                acceleration[2].toDouble(),
                rate[0].toDouble(),
                rate[1].toDouble(),
                rate[2].toDouble(),
            )
        } catch (_: Exception) {
            return
        }
        onImu(reading)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private fun depthFrame(
        frame: Frame,
        translation: FloatArray,
        rotation: FloatArray,
        timestamp: Double,
    ): MappingDepthFrame? {
        val depth = try {
            frame.acquireRawDepthImage16Bits()
        } catch (_: NotYetAvailableException) {
            return null
        }
        val confidence = try {
            frame.acquireRawDepthConfidenceImage()
        } catch (_: NotYetAvailableException) {
            depth.close()
            return null
        }
        depth.use { image ->
            confidence.use { trust ->
                val width = image.width
                val height = image.height
                if (width <= 0 || height <= 0 || trust.width != width || trust.height != height) return null
                val depthPlane = image.planes[0]
                val trustPlane = trust.planes[0]
                val values = depth16ToMetres(
                    bytes(depthPlane.buffer),
                    depthPlane.rowStride.toUInt(),
                    bytes(trustPlane.buffer),
                    trustPlane.rowStride.toUInt(),
                    width.toUInt(),
                    height.toUInt(),
                    defaultMinDepthConfidence(),
                )
                val intrinsics = frame.camera.imageIntrinsics
                val focal = FloatArray(2)
                val principal = FloatArray(2)
                intrinsics.getFocalLength(focal, 0)
                intrinsics.getPrincipalPoint(principal, 0)
                val dims = intrinsics.imageDimensions
                val sx = width.toDouble() / dims[0].toDouble()
                val sy = height.toDouble() / dims[1].toDouble()
                val optical = yUpCameraToOpticalPose(
                    translation[0].toDouble(),
                    translation[1].toDouble(),
                    translation[2].toDouble(),
                    rotation[0].toDouble(),
                    rotation[1].toDouble(),
                    rotation[2].toDouble(),
                    rotation[3].toDouble(),
                )
                // A measured mount height puts the floor at Z = 0; otherwise assume 0.5 m.
                groundOffset = latchGroundOffset(groundOffset, optical.z, mountHeight() ?: 0.5)
                val offset = groundOffset ?: 0.0
                return MappingDepthFrame(
                    timestamp = timestamp,
                    width = width.toUInt(),
                    height = height.toUInt(),
                    fx = focal[0].toDouble() * sx,
                    fy = focal[1].toDouble() * sy,
                    cx = (principal[0].toDouble() + 0.5) * sx - 0.5,
                    cy = (principal[1].toDouble() + 0.5) * sy - 0.5,
                    cameraX = optical.x,
                    cameraY = optical.y,
                    cameraZ = optical.z + offset,
                    quaternionX = optical.quaternionX,
                    quaternionY = optical.quaternionY,
                    quaternionZ = optical.quaternionZ,
                    quaternionW = optical.quaternionW,
                    depthMetres = values,
                )
            }
        }
    }

    private fun bytes(buffer: ByteBuffer): ByteArray {
        val copy = buffer.duplicate()
        copy.rewind()
        return ByteArray(copy.remaining()).also { copy.get(it) }
    }

    companion object {
        fun monotonicSeconds(): Double = SystemClock.elapsedRealtimeNanos().toDouble() / 1_000_000_000.0
    }
}

/** A 16×16 pbuffer so ARCore can publish the camera without a visible GL surface. */
private class HeadlessEgl {
    private var display = EGL14.EGL_NO_DISPLAY
    private var context = EGL14.EGL_NO_CONTEXT
    private var surface = EGL14.EGL_NO_SURFACE
    var textureId: Int = 0
        private set

    fun start(): Boolean {
        display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        if (display == EGL14.EGL_NO_DISPLAY) return false
        val version = IntArray(2)
        if (!EGL14.eglInitialize(display, version, 0, version, 1)) return false
        val attribs = intArrayOf(
            EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
            EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_NONE,
        )
        val configs = arrayOfNulls<EGLConfig>(1)
        val count = IntArray(1)
        if (!EGL14.eglChooseConfig(display, attribs, 0, configs, 0, 1, count, 0) || count[0] == 0) {
            return false
        }
        val contextAttribs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
        context = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT, contextAttribs, 0)
        if (context == EGL14.EGL_NO_CONTEXT) return false
        val pbuffer = intArrayOf(EGL14.EGL_WIDTH, 16, EGL14.EGL_HEIGHT, 16, EGL14.EGL_NONE)
        surface = EGL14.eglCreatePbufferSurface(display, configs[0], pbuffer, 0)
        if (surface == EGL14.EGL_NO_SURFACE) return false
        if (!EGL14.eglMakeCurrent(display, surface, surface, context)) return false
        val textures = IntArray(1)
        GLES20.glGenTextures(1, textures, 0)
        textureId = textures[0]
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR)
        GLES20.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR)
        return textureId != 0
    }

    fun close() {
        if (display != EGL14.EGL_NO_DISPLAY) {
            EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
            if (surface != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, surface)
            if (context != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, context)
            EGL14.eglTerminate(display)
        }
        display = EGL14.EGL_NO_DISPLAY
        context = EGL14.EGL_NO_CONTEXT
        surface = EGL14.EGL_NO_SURFACE
        textureId = 0
    }
}
