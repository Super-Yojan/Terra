package dev.superyojan.terra.phone

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider

class TerraApp : android.app.Application() {
    override fun onCreate() {
        super.onCreate()
        // Load before JNA's first lookup so the linker sees the packaged library.
        System.loadLibrary("terra_mobile")
    }
}

class MainActivity : ComponentActivity() {
    private lateinit var session: TerraSession
    private val cameraPermission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        if (granted) session.startPhone()
    }
    private val bluetoothPermission = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { granted ->
        if (granted.values.all { it }) session.scanBluetooth() else session.ble?.permissionDenied()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        session = TerraSession(this)
        session.discardZenohOnDevice()
        setContent {
            TerraTheme {
                ControllerScreen(
                    session = session,
                    onStartPhone = {
                        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
                            session.startPhone()
                        } else {
                            cameraPermission.launch(Manifest.permission.CAMERA)
                        }
                    },
                    onScanBluetooth = { requestBluetoothScan() },
                    onShareLog = { shareRunLog() },
                )
            }
        }
    }

    private fun requestBluetoothScan() {
        val permissions = if (Build.VERSION.SDK_INT >= 31) {
            arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        val missing = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isEmpty()) session.scanBluetooth() else bluetoothPermission.launch(missing.toTypedArray())
    }

    /** Hands the current run's JSONL log to any app that takes a file. */
    private fun shareRunLog() {
        val log = session.runLog?.takeIf { it.isFile } ?: return
        val uri = FileProvider.getUriForFile(this, "$packageName.runlogs", log)
        val send = Intent(Intent.ACTION_SEND)
            .setType("application/json")
            .putExtra(Intent.EXTRA_STREAM, uri)
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        startActivity(Intent.createChooser(send, "Share run log"))
    }

    override fun onStop() {
        if (::session.isInitialized) session.stop()
        super.onStop()
    }

    override fun onDestroy() {
        if (::session.isInitialized) session.close()
        super.onDestroy()
    }
}
