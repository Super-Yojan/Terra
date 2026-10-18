plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose")
}

android {
    namespace = "dev.superyojan.terra"
    compileSdk = 35

    defaultConfig {
        applicationId = "dev.superyojan.terra"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
        ndk {
            abiFilters += listOf("arm64-v8a", "x86_64")
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro",
            )
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        compose = true
    }

    packaging {
        jniLibs {
            // JNA opens the library by name after the package manager extracts it.
            useLegacyPackaging = true
        }
    }

    testOptions {
        unitTests.isReturnDefaultValues = true
    }
}

dependencies {
    val composeBom = platform("androidx.compose:compose-bom:2024.10.01")
    implementation(composeBom)
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.activity:activity-compose:1.9.3")
    implementation("androidx.core:core-ktx:1.15.0")
    implementation("net.java.dev.jna:jna:5.17.0@aar")
    implementation("com.google.ar:core:1.46.0")

    testImplementation("junit:junit:4.13.2")
    testImplementation("net.java.dev.jna:jna:5.17.0")
}

val hostLibDir = rootProject.file("../../target/debug")
tasks.withType<Test>().configureEach {
    systemProperty("jna.library.path", hostLibDir.absolutePath)
    environment("LD_LIBRARY_PATH", listOfNotNull(hostLibDir.absolutePath, System.getenv("LD_LIBRARY_PATH")).joinToString(":"))
    val zenoh = System.getenv("TERRA_ZENOH_SMOKE_ENDPOINT")
    if (!zenoh.isNullOrBlank()) {
        environment("TERRA_ZENOH_SMOKE_ENDPOINT", zenoh)
    }
}
