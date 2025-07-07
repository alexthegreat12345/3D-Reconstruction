// TeamCode/src/test/java/org/firstinspires/ftc/teamcode/Reconstruction3D/CalibrationManagerTest.java
package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.junit.AssumptionViolatedException;
import org.junit.BeforeClass;
import org.junit.Test;
import org.opencv.core.Core; // For OpenCV version and loading

public class CalibrationManagerTest {

    @BeforeClass
    public static void setUpClass() {
        // Attempt to load OpenCV native library
        // This path needs to be configured correctly for your desktop environment
        // Option 1: System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        // This requires the native library to be in a path known to the JVM
        // (e.g., via -Djava.library.path or system PATH/LD_LIBRARY_PATH)

        // Option 2: Load explicitly if you know the exact path (less portable)
        // System.load("/path/to/your/opencv_javaXXX.dylib"); // or .so or .dll

        // For simplicity with JUnit, often the -Djava.library.path VM option is preferred
        // when running the test.
        try {
//            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            System.load("/Users/hao.wu/opencv_macos_install/share/java/opencv4/libopencv_java4110.dylib"); // or .so or .dll

            System.out.println("OpenCV Native Library loaded successfully: " + Core.NATIVE_LIBRARY_NAME);
            System.out.println("OpenCV Version: " + Core.VERSION);
        } catch (UnsatisfiedLinkError e) {
//            System.err.println("Native code library failed to load.\n" +
//                    "Ensure OpenCV native libraries are in your java.library.path.\n" + e);
            // You might want to throw an AssumptionViolatedException here if OpenCV is critical for tests
            // import org.junit.AssumptionViolatedException;
            // throw new AssumptionViolatedException("OpenCV not available, skipping tests.", e);
//            System.exit(1); // Or handle more gracefully
            String errorMessage = "OpenCV Native code library failed to load.\n" +
                    "Ensure OpenCV native libraries are in your java.library.path.\n" + e.getMessage();
            System.err.println(errorMessage);
            throw new AssumptionViolatedException(errorMessage, e); // Good change
        }
    }

    @Test
    public void runCalibrationExampleLocally() {
        System.out.println("=== Starting Local Calibration Test ===");

        // --- CRITICAL: Configure Paths for Local Execution ---
        // Temporarily override or ensure CalibrationManager uses local paths
        // For this test, we'll assume CalibrationManager's paths are already set
        // to your local desktop paths (e.g., /Users/hao.wu/Documents/calib).
        // If not, you might need a way to pass these paths to CalibrationManager
        // or modify it to accept them for testing.

        CalibrationManager calibrationManager = new CalibrationManager();

        // Call the method you want to test
        // Note: exampleFTCUsage() logs to android.util.Log.
        // This will work, but output might go to console differently than in an Android app.
        // Standard System.out.println will also work for logging in tests.
        calibrationManager.exampleFTCUsage();

        // You can add assertions here if your method returns values or has side effects
        // you want to verify. For example, if performCompleteCalibration returned a boolean:
        // boolean success = calibrationManager.performCompleteCalibration(); // Assuming you modify to return
        // assertTrue("Calibration should be successful", success);

        System.out.println("=== Local Calibration Test Finished ===");

    }
}
