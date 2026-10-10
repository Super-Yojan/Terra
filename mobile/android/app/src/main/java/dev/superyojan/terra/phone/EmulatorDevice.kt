package dev.superyojan.terra.phone

import android.os.Build

/** Emulator builds show Zenoh. A physical phone keeps that section out, matching the iOS split. */
fun isEmulator(): Boolean {
    val fingerprint = Build.FINGERPRINT
    val model = Build.MODEL
    val hardware = Build.HARDWARE
    val product = Build.PRODUCT
    return fingerprint.startsWith("generic")
        || fingerprint.startsWith("unknown")
        || fingerprint.contains("emulator")
        || model.contains("google_sdk")
        || model.contains("Emulator")
        || model.contains("Android SDK built for")
        || hardware.contains("goldfish")
        || hardware.contains("ranchu")
        || product.contains("sdk")
        || product.contains("emulator")
        || product.contains("simulator")
        || Build.MANUFACTURER.contains("Genymotion")
        || (Build.BRAND.startsWith("generic") && Build.DEVICE.startsWith("generic"))
}
