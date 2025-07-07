package org.firstinspires.ftc.teamcode.Reconstruction3D;
import org.opencv.core.Mat;
import org.opencv.android.Utils;
import org.opencv.imgcodecs.Imgcodecs;

import android.graphics.Bitmap;
import android.util.Log;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Example usage of Single Camera Calibration and Stereo Camera Calibration
 * for FTC Stereo Vision System
 */
public class CalibrationManager {
    private static final String TAG = "CalibrationExample";

    // Path to store calibration data files
//    private static final String CALIBRATION_DATA_PATH = "/storage/emulated/0/FTC/calibration/";
    private static final String CALIBRATION_DATA_PATH = "/Users/hao.wu/Documents/calib";

    // Camera identifiers
    private static final String LEFT_CAMERA_NAME = "left";
    private static final String RIGHT_CAMERA_NAME = "right";

    private SingleCameraCalibration singleCalibration;
//    private StereoCameraCalibration stereoCalibration;

    public CalibrationManager() {
        singleCalibration = new SingleCameraCalibration(CALIBRATION_DATA_PATH);
//        stereoCalibration = new StereoCameraCalibration(CALIBRATION_DATA_PATH);
    }

    /**
     * STEP 1: Calibrate individual cameras
     * Call this method for both left and right cameras separately
     */
    public boolean performSingleCameraCalibration() {
        Log.d(TAG, "=== Starting Single Camera Calibration ===");

        // Step 1a: Calibrate LEFT camera
        Log.d(TAG, "Calibrating LEFT camera...");
        List<Mat> leftCalibrationImages = loadCalibrationImages("left");

        if (leftCalibrationImages == null || leftCalibrationImages.isEmpty()) {
            Log.e(TAG, "No left camera calibration images found");
            return false;
        }

        SingleCameraCalibration.CameraCalibrationData leftCalibData =
                singleCalibration.calibrateCamera(leftCalibrationImages, LEFT_CAMERA_NAME);

        if (leftCalibData == null || !leftCalibData.isValid()) {
            Log.e(TAG, "Left camera calibration failed");
            return false;
        }

        Log.d(TAG, "Left camera calibration successful! RMS: " + leftCalibData.rms);

        // Step 1b: Calibrate RIGHT camera
        Log.d(TAG, "Calibrating RIGHT camera...");
        List<Mat> rightCalibrationImages = loadCalibrationImages("right");

        if (rightCalibrationImages == null || rightCalibrationImages.isEmpty()) {
            Log.e(TAG, "No right camera calibration images found");
            return false;
        }

        SingleCameraCalibration.CameraCalibrationData rightCalibData =
                singleCalibration.calibrateCamera(rightCalibrationImages, RIGHT_CAMERA_NAME);

        if (rightCalibData == null || !rightCalibData.isValid()) {
            Log.e(TAG, "Right camera calibration failed");
            return false;
        }

        Log.d(TAG, "Right camera calibration successful! RMS: " + rightCalibData.rms);
        Log.d(TAG, "=== Single Camera Calibration Complete ===");

        return true;
    }

//    /**
//     * STEP 2: Calibrate stereo camera system
//     * This uses the results from Step 1
//     */
//    public boolean performStereoCalibration() {
//        Log.d(TAG, "=== Starting Stereo Camera Calibration ===");
//
//        // Check if single camera calibrations exist
//        if (!singleCalibration.hasCalibrationData(LEFT_CAMERA_NAME) ||
//                !singleCalibration.hasCalibrationData(RIGHT_CAMERA_NAME)) {
//            Log.e(TAG, "Single camera calibrations not found. Please run Step 1 first.");
//            return false;
//        }
//
//        // Load stereo calibration image pairs
//        List<Mat> leftStereoImages = loadStereoImages("left");
//        List<Mat> rightStereoImages = loadStereoImages("right");
//
//        if (leftStereoImages == null || rightStereoImages == null ||
//                leftStereoImages.isEmpty() || rightStereoImages.isEmpty()) {
//            Log.e(TAG, "No stereo calibration image pairs found");
//            return false;
//        }
//
//        if (leftStereoImages.size() != rightStereoImages.size()) {
//            Log.e(TAG, "Stereo image pair count mismatch: " +
//                    leftStereoImages.size() + " vs " + rightStereoImages.size());
//            return false;
//        }
//
//        // Perform stereo calibration
//        StereoCameraCalibration.StereoCalibrationData stereoData =
//                stereoCalibration.calibrateStereoCamera(
//                        leftStereoImages, rightStereoImages,
//                        LEFT_CAMERA_NAME, RIGHT_CAMERA_NAME);
//
//        if (stereoData == null || !stereoData.isValid()) {
//            Log.e(TAG, "Stereo calibration failed");
//            return false;
//        }
//
//        Log.d(TAG, "Stereo calibration successful! RMS: " + stereoData.rms);
//
//        // Log baseline distance
//        double baseline = StereoCameraCalibration.getBaseline(stereoData);
//        Log.d(TAG, "Baseline distance: " + baseline + " mm");
//
//        Log.d(TAG, "=== Stereo Camera Calibration Complete ===");
//
//        return true;
//    }

    /**
     * Complete calibration workflow (Step 1 + Step 2)
     */
    public boolean performCompleteCalibration() {
        Log.d(TAG, "=== Starting Complete Calibration Workflow ===");

        // Step 1: Single camera calibration
        if (!performSingleCameraCalibration()) {
            Log.e(TAG, "Single camera calibration failed");
            return false;
        }

//        // Step 2: Stereo calibration
//        if (!performStereoCalibration()) {
//            Log.e(TAG, "Stereo calibration failed");
//            return false;
//        }

        Log.d(TAG, "=== Complete Calibration Workflow Finished Successfully ===");
        return true;
    }

//    /**
//     * Validate calibration by checking if all required data exists
//     */
//    public boolean validateCalibration() {
//        Log.d(TAG, "=== Validating Calibration Data ===");
//
//        // Check single camera calibrations
//        boolean leftExists = singleCalibration.hasCalibrationData(LEFT_CAMERA_NAME);
//        boolean rightExists = singleCalibration.hasCalibrationData(RIGHT_CAMERA_NAME);
//        boolean stereoExists = stereoCalibration.hasStereoCalibrationData();
//
//        Log.d(TAG, "Left camera calibration: " + (leftExists ? "EXISTS" : "MISSING"));
//        Log.d(TAG, "Right camera calibration: " + (rightExists ? "EXISTS" : "MISSING"));
//        Log.d(TAG, "Stereo calibration: " + (stereoExists ? "EXISTS" : "MISSING"));
//
//        if (leftExists && rightExists && stereoExists) {
//            // Load and validate the actual data
//            SingleCameraCalibration.CameraCalibrationData leftData =
//                    singleCalibration.loadCalibrationData(LEFT_CAMERA_NAME);
//            SingleCameraCalibration.CameraCalibrationData rightData =
//                    singleCalibration.loadCalibrationData(RIGHT_CAMERA_NAME);
//            StereoCameraCalibration.StereoCalibrationData stereoData =
//                    stereoCalibration.loadStereoCalibrationData();
//
//            boolean allValid = (leftData != null && leftData.isValid()) &&
//                    (rightData != null && rightData.isValid()) &&
//                    (stereoData != null && stereoData.isValid());
//
//            Log.d(TAG, "Calibration validation: " + (allValid ? "PASSED" : "FAILED"));
//
//            if (allValid) {
//                Log.d(TAG, "Left camera RMS: " + leftData.rms);
//                Log.d(TAG, "Right camera RMS: " + rightData.rms);
//                Log.d(TAG, "Stereo RMS: " + stereoData.rms);
//                Log.d(TAG, "Baseline: " + StereoCameraCalibration.getBaseline(stereoData) + " mm");
//            }
//
//            return allValid;
//        }
//
//        return false;
//    }

    /**
     * Test chessboard detection on a single image
     */
    public boolean testChessboardDetection(Mat testImage, String cameraName) {
        Log.d(TAG, "Testing chessboard detection for " + cameraName + " camera");

        boolean detected = singleCalibration.validateChessboardDetection(testImage);

        Log.d(TAG, "Chessboard detection result: " + (detected ? "SUCCESS" : "FAILED"));
        return detected;
    }

//    /**
//     * Test stereo chessboard detection on image pair
//     */
//    public boolean testStereoChessboardDetection(Mat leftImage, Mat rightImage) {
//        Log.d(TAG, "Testing stereo chessboard detection");
//
//        boolean detected = stereoCalibration.validateStereoChessboardDetection(leftImage, rightImage);
//
//        Log.d(TAG, "Stereo chessboard detection result: " + (detected ? "SUCCESS" : "FAILED"));
//        return detected;
//    }

    /**
     * Load calibration images for single camera calibration
     * In practice, you would load these from your camera or files
     */
    private List<Mat> loadCalibrationImages(String cameraName) {
        List<Mat> images = new ArrayList<>();
        String directoryPath;

        // Determine the directory path based on the camera name
        if (LEFT_CAMERA_NAME.equals(cameraName)) {
            // You can customize this path for the left camera if needed
            directoryPath = "/Users/hao.wu/Documents/calib/left/";
            Log.d(TAG, "Loading LEFT camera calibration images from: " + directoryPath);
        } else if (RIGHT_CAMERA_NAME.equals(cameraName)) {
            directoryPath = "/Users/hao.wu/Documents/calib/right/";
            Log.d(TAG, "Loading RIGHT camera calibration images from: " + directoryPath);
        } else {
            Log.e(TAG, "Unknown camera name for loading calibration images: " + cameraName);
            return images; // Return empty list
        }

        File imageDir = new File(directoryPath);
        if (!imageDir.exists() || !imageDir.isDirectory()) {
            Log.e(TAG, "Calibration image directory does not exist or is not a directory: " + directoryPath);
            return images; // Return empty list
        }

        File[] imageFiles = imageDir.listFiles((dir, name) ->
                name.toLowerCase().endsWith(".jpg") ||
                        name.toLowerCase().endsWith(".jpeg") ||
                        name.toLowerCase().endsWith(".png")
        );

        if (imageFiles == null || imageFiles.length == 0) {
            Log.w(TAG, "No image files found in directory: " + directoryPath);
            return images; // Return empty list
        }

        Log.d(TAG, "Found " + imageFiles.length + " potential calibration images in " + directoryPath);

        for (File imageFile : imageFiles) {
            Mat img = Imgcodecs.imread(imageFile.getAbsolutePath());
            if (!img.empty()) {
                images.add(img);
                Log.d(TAG, "Successfully loaded image: " + imageFile.getName());
            } else {
                Log.w(TAG, "Failed to load image or image is empty: " + imageFile.getName());
            }
        }

        if (images.isEmpty()) {
            Log.w(TAG, "No images were successfully loaded from " + directoryPath);
        } else {
            Log.i(TAG, "Successfully loaded " + images.size() + " calibration images for " + cameraName + " camera.");
        }

        return images;
    }

    /**
     * Load stereo calibration image pairs
     * In practice, you would load synchronized pairs from both cameras
     */
    private List<Mat> loadStereoImages(String cameraName) {
        List<Mat> images = new ArrayList<>();

        // TODO: Implement your stereo image loading logic here
        // Important: Left and right image lists must be synchronized
        // (same index = same time capture)

        Log.d(TAG, "Loading " + cameraName + " stereo calibration images...");

        // Placeholder - you need to implement actual image loading
        Log.w(TAG, "Stereo image loading not implemented - returning empty list");

        return images;
    }

    /**
     * Helper method to convert Bitmap to Mat (if you're loading from Android resources)
     */
    private Mat bitmapToMat(Bitmap bitmap) {
        Mat mat = new Mat();
        Utils.bitmapToMat(bitmap, mat);
        return mat;
    }

    /**
     * Example of how to use this in your FTC OpMode
     */
    public void exampleFTCUsage() {
        Log.d(TAG, "=== FTC OpMode Example ===");

        // During initialization or setup phase
        CalibrationManager calibrationExample = new CalibrationManager();
        boolean success = calibrationExample.performCompleteCalibration();


//        // Check if calibration already exists
//        if (calibrationExample.validateCalibration()) {
//            Log.d(TAG, "Calibration data found and validated - ready for 3D vision");
//            // Proceed to object detection and 3D coordinate calculation
//        } else {
//            Log.d(TAG, "Calibration data missing or invalid");
//
//            // Option 1: Run calibration now (if you have calibration images)
//            // boolean success = calibrationExample.performCompleteCalibration();
//
//            // Option 2: Load pre-calibrated data
//            // boolean success = loadPreCalibratedData();
//
//            // Option 3: Inform user that calibration is needed
//            Log.w(TAG, "Please run camera calibration before using stereo vision");
//        }
    }
}