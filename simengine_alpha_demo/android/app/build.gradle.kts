// android/app/build.gradle.kts
//
// Builds the Alpha Technical Demo as a plain NativeActivity APK — no
// Kotlin/Java application code at all (see AndroidManifest.xml: the
// activity IS android.app.NativeActivity, pointed at libsimengine_android.so
// via the android.app.lib_name meta-data). Everything interactive lives
// in the native code under src/main/cpp/, built by externalNativeBuild
// below, which in turn just add_subdirectory()s the project's own root
// CMakeLists.txt — see src/main/cpp/CMakeLists.txt's header comment.

plugins {
    id("com.android.application")
}

android {
    namespace = "com.simengine.alphademo"
    compileSdk = 34
    ndkVersion = "26.3.11579264"

    defaultConfig {
        applicationId = "com.simengine.alphademo"
        minSdk = 26   // NativeActivity+GLES3 work fine much lower, but the
                       // adaptive vector launcher icon (see res/mipmap-anydpi-v26)
                       // needs 26+; raise this back down together with adding
                       // legacy PNG mipmaps if you need to support older devices.
        targetSdk = 34
        versionCode = 1
        versionName = "0.1.0-alpha"

        ndk {
            // arm64-v8a covers effectively all real Android hardware from
            // the last ~7 years; armeabi-v7a kept for older/low-end
            // devices since this Alpha has no reason to exclude them yet.
            abiFilters += listOf("arm64-v8a", "armeabi-v7a")
        }

        externalNativeBuild {
            cmake {
                cppFlags += listOf("-std=c++20")
                arguments += listOf(
                    "-DSIMENGINE_ROOT=${file("$projectDir/../..").absolutePath}",
                    "-DANDROID_STL=c++_shared"
                )
            }
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
        debug {
            isJniDebuggable = true
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    // No Java/Kotlin sources — NativeActivity needs none. Keeping the
    // srcSets minimal (manifest + res + the native cpp/ tree CMake reads
    // directly) avoids Gradle expecting a src/main/java that doesn't
    // exist.
    //
    // assets.srcDirs points straight at the repo's own assets/ folder
    // (the converted A320 meshes + JSBSim FDM data — see
    // android_main.cpp's extractAssetsIfNeeded()) rather than
    // duplicating those files under android/app/src/main/assets/, so
    // there's one copy to keep in sync, not two.
    sourceSets {
        getByName("main") {
            manifest.srcFile("src/main/AndroidManifest.xml")
            res.srcDirs("src/main/res")
            assets.srcDirs("../../assets")
        }
    }

    packaging {
        // android_native_app_glue + our code link against the shared
        // libc++ (ANDROID_STL=c++_shared above); make sure it actually
        // ends up in the APK for every ABI we build.
        jniLibs.useLegacyPackaging = false
    }
}
