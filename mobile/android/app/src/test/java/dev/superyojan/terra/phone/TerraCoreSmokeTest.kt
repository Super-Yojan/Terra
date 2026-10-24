package dev.superyojan.terra.phone

import dev.superyojan.terra.core.ActuatorAssembler
import dev.superyojan.terra.core.ImuReading
import dev.superyojan.terra.core.actuatorEncodeControl
import dev.superyojan.terra.core.actuatorEncodeFrame
import dev.superyojan.terra.core.actuatorEncodeRelayDrive
import dev.superyojan.terra.core.actuatorFragment
import dev.superyojan.terra.core.actuatorRouteProfiles
import dev.superyojan.terra.core.actuatorRouteStick
import dev.superyojan.terra.core.MappingDepthFrame
import dev.superyojan.terra.core.MobileController
import dev.superyojan.terra.core.MobileOccupancyMap
import dev.superyojan.terra.core.MobileWaypoint
import dev.superyojan.terra.core.MobileZenohClient
import dev.superyojan.terra.core.SafetyState
import dev.superyojan.terra.core.TwistSetpoint
import dev.superyojan.terra.core.VioReading
import dev.superyojan.terra.core.WaypointPhase
import dev.superyojan.terra.core.defaultControlSettings
import dev.superyojan.terra.core.defaultOccupancySettings
import dev.superyojan.terra.core.defaultWaypointSettings
import dev.superyojan.terra.core.deviceVectorToBody
import dev.superyojan.terra.core.runVelocityBenchmark
import dev.superyojan.terra.core.supportedAutonomyLevels
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Kotlin → UniFFI → Rust. The host `libterra_mobile.so` is on `jna.library.path`
 * (see `app/build.gradle.kts`). Set `TERRA_ZENOH_SMOKE_ENDPOINT` to also drive a peer.
 */
class TerraCoreSmokeTest {
    @Test
    fun controllerStepsWaypointAndBenchmark() {
        val brain = MobileController(defaultControlSettings())
        brain.pushImu(ImuReading(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        brain.pushVio(
            VioReading(
                timestamp = 0.0,
                positionX = 0.0,
                positionY = 0.0,
                positionZ = 0.0,
                quaternionX = 0.0,
                quaternionY = 0.0,
                quaternionZ = 0.0,
                quaternionW = 1.0,
                velocityX = 0.0,
                velocityY = 0.0,
                velocityZ = 0.0,
                tracked = true,
            ),
        )
        brain.setTarget(TwistSetpoint(timestamp = 0.0, forward = 1.0, yawRate = 0.3))
        val output = brain.step(0.0)
        assertEquals(SafetyState.ACTIVE, output.safety)
        assertTrue(output.rightEffort > output.leftEffort)
        val stale = brain.step(1.0)
        assertEquals(0.0, stale.leftEffort, 0.0)
        assertEquals(0.0, stale.rightEffort, 0.0)

        val forward = deviceVectorToBody(0.0, 1.0, 0.0)
        assertEquals(1.0, forward.x, 1e-9)
        assertEquals(0.0, forward.y, 1e-9)

        val follower = MobileWaypoint(defaultWaypointSettings())
        follower.setOrigin(38.8297, -77.3075)
        assertTrue(follower.setGoal(38.82981, -77.3075, null, "gmu-north", 49.0))
        val step = follower.step(0.0, 0.0, 0.0)
        assertEquals(WaypointPhase.ACTIVE, step.phase)
        assertTrue("expected a forward command, got ${step.forward}", step.forward > 0.5)

        val levels = supportedAutonomyLevels()
        assertTrue(levels.contains("teleop"))
        assertTrue(levels.contains("waypoint"))
        assertTrue(levels.contains("explore"))

        val report = runVelocityBenchmark()
        assertTrue(report.passed)

        val endpoint = System.getenv("TERRA_ZENOH_SMOKE_ENDPOINT")
        if (!endpoint.isNullOrEmpty()) {
            driveZenohPeer(endpoint)
        }
        println(
            "Kotlin → UniFFI → Rust passed: v=%.3f m/s, yaw=%.3f rad/s".format(
                report.finalForward,
                report.finalYawRate,
            ),
        )
    }

    @Test
    fun actuatorFramingRoundTrip() {
        val routed = actuatorRouteStick(DRIVE_LAYOUT, 0.2f, 0.5f)
        val relay = actuatorEncodeRelayDrive(DRIVE_LAYOUT, routed)
        assertEquals(4, relay.size)
        assertEquals(0.toByte(), relay[0])
        assertEquals((-30).toByte(), relay[1])
        assertEquals(1.toByte(), relay[2])
        assertEquals(70.toByte(), relay[3])

        val profiles = actuatorRouteProfiles(DRIVE_PROFILES, 0.2f, 0.5f)
        val fromProfiles = actuatorEncodeRelayDrive(DRIVE_PROFILES, profiles)
        assertTrue(fromProfiles.contentEquals(relay))

        val disarm = actuatorEncodeFrame(
            """{"kind":"disarm","session":7,"revision":1,"sequence":1,"values":[]}""",
        )
        assertEquals(16, disarm.size)
        assertEquals('T'.code.toByte(), disarm[0])
        assertEquals('A'.code.toByte(), disarm[1])
        assertEquals(1.toByte(), disarm[2])
        assertEquals(3.toByte(), disarm[3])

        val payload = "hello".toByteArray()
        val parts = actuatorFragment(7.toUShort(), payload, 7u)
        assertEquals(2, parts.size)
        val assembler = ActuatorAssembler(96u)
        assertEquals(null, assembler.push(parts[0], 0uL))
        val done = checkNotNull(assembler.push(parts[1], 1uL))
        assertTrue(payload.contentEquals(done))
        assembler.close()

        val control = actuatorEncodeControl(
            """{"schema_version":1,"request_id":1000,"operation":"capabilities","payload":{}}""",
        )
        val text = control.toString(Charsets.UTF_8)
        assertTrue(text.contains("capabilities"))
        assertTrue(text.contains("1000"))
    }
}

private const val DRIVE_LAYOUT = """{"schema_version":1,"revision":1,"actuators":[
    {"id":0,"name":"left","port":"M0","kind":"dc_motor","inverted":false,
     "limits":{"min":-1.0,"max":1.0},
     "calibration":{"type":"dc_motor","max_power_fraction":1.0},
     "route":{"type":"left_effort"},"safe":{"type":"zero"}},
    {"id":1,"name":"right","port":"M1","kind":"dc_motor","inverted":false,
     "limits":{"min":-1.0,"max":1.0},
     "calibration":{"type":"dc_motor","max_power_fraction":1.0},
     "route":{"type":"right_effort"},"safe":{"type":"zero"}}
]}"""

private const val DRIVE_PROFILES = """{"schema_version":1,"revision":1,"actuators":[
    {"id":0,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"left_effort"},"safe":{"type":"zero"}},
    {"id":1,"kind":"dc_motor","limits":{"min":-1.0,"max":1.0},"route":{"type":"right_effort"},"safe":{"type":"zero"}}
]}"""

private fun driveZenohPeer(endpoint: String) {
    val remote = MobileZenohClient(endpoint, "terra/rover", 9u)
    val deadline = System.nanoTime() + 3_000_000_000L
    var mapped = false
    while (System.nanoTime() < deadline) {
        val frame = remote.takeDepth()
        if (frame != null) {
            assertEquals(0.5, frame.cameraZ, 1e-6)
            assertEquals(0.0, frame.bodyYaw, 1e-6)
            assertEquals(1u, frame.width)
            assertEquals(1, frame.depthMetres.size)
            val map = MobileOccupancyMap(defaultOccupancySettings())
            map.recenter(frame.bodyX, frame.bodyY)
            map.integrateDepth(
                MappingDepthFrame(
                    timestamp = frame.timestamp,
                    width = frame.width,
                    height = frame.height,
                    fx = frame.fx,
                    fy = frame.fy,
                    cx = frame.cx,
                    cy = frame.cy,
                    cameraX = frame.cameraX,
                    cameraY = frame.cameraY,
                    cameraZ = frame.cameraZ,
                    quaternionX = frame.quaternionX,
                    quaternionY = frame.quaternionY,
                    quaternionZ = frame.quaternionZ,
                    quaternionW = frame.quaternionW,
                    depthMetres = frame.depthMetres,
                ),
            )
            assertTrue(map.snapshot().occupancy.any { it > 50 })
            mapped = true
            break
        }
        Thread.sleep(50)
    }
    assertTrue("posed simulator depth did not integrate into an occupied cell", mapped)
    repeat(20) {
        remote.setTarget(0.5, 0.2)
        Thread.sleep(50)
    }
    // Stop refreshing: the Rust lease must send zero while the connection stays open.
    Thread.sleep(400)
    remote.disconnect()
    assertEquals("Disconnected", remote.status())
    println("Kotlin → UniFFI → Zenoh network path passed")
}
