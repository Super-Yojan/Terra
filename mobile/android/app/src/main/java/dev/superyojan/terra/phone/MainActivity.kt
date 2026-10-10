package dev.superyojan.terra.phone

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat

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
                )
            }
        }
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
