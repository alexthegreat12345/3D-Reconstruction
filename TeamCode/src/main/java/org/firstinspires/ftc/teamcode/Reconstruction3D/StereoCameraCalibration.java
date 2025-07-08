package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import android.util.Log;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.Serializable;

/**
 * Stereo Camera Calibration for FTC Stereo Vision System
 * Step 2: Calibrates stereo camera system using pre-saved single camera calibrations
 */
public class StereoCameraCalibration {
    private static final String TAG = "StereoCameraCalibration";

    // Chessboard pattern size (must match SingleCameraCalibration)
    private static final int CHESSBOARD_WIDTH = 9;
    private static final int CHESSBOARD_HEIGHT = 6;
    private static final Size CHESSBOARD_SIZE = new Size(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT);

    // Square size in real world units (must match SingleCameraCalibration)
    private static final float SQUARE_SIZE = 25.0f;

    /**
     * Stereo calibration data storage class
     */
    public static class StereoCalibrationData implements Serializable {
        public Mat R;  // Rotation matrix between left and right camera
        public Mat T;  // Translation vector between left and right camera
        public Mat E;  // Essential matrix
        public Mat F;  // Fundamental matrix
        public Mat R1, R2;  // Rectification transforms for left and right camera
        public Mat P1, P2;  // Projection matrices for left and right camera
        public Mat Q;  // Disparity-to-depth mapping matrix (4x4)
        public double rms;  // RMS reprojection error
        public Size imageSize;

        public StereoCalibrationData() {
            R = new Mat(3, 3, CvType.CV_64F);
            T = new Mat(3, 1, CvType.CV_64F);
            E = new Mat(3, 3, CvType.CV_64F);
            F = new Mat(3, 3, CvType.CV_64F);
            R1 = new Mat(3, 3, CvType.CV_64F);
            R2 = new Mat(3, 3, CvType.CV_64F);
            P1 = new Mat(3, 4, CvType.CV_64F);
            P2 = new Mat(3, 4, CvType.CV_64F);
            Q = new Mat(4, 4, CvType.CV_64F);
        }

        public boolean isValid() {
            return R != null && !R.empty() &&
                    T != null && !T.empty() &&
                    Q != null && !Q.empty();
        }
    }

    // File path for saving calibration data
    private String calibrationDataPath;
    private SingleCameraCalibration singleCameraCalibration;

    public StereoCameraCalibration(String dataPath) {
        this.calibrationDataPath = dataPath;
        this.singleCameraCalibration = new SingleCameraCalibration(dataPath);

        // Create directory if it doesn't exist
        File dir = new File(dataPath);
        if (!dir.exists()) {
            dir.mkdirs();
        }
    }

    /**
     * Calibrate stereo camera system using stereo image pairs
     *
     * @param leftImages List of left camera images with chessboard pattern
     * @param rightImages List of right camera images with chessboard pattern
     * @param leftCameraName Name identifier for left camera (e.g., "left")
     * @param rightCameraName Name identifier for right camera (e.g., "right")
     * @return StereoCalibrationData containing stereo calibration results
     */
    public StereoCalibrationData calibrateStereoCamera(List<Mat> leftImages, List<Mat> rightImages,
                                                       String leftCameraName, String rightCameraName) {
        Log.d(TAG, "Starting stereo camera calibration");

        // Validate input
        if (leftImages == null || rightImages == null || leftImages.isEmpty() || rightImages.isEmpty()) {
            Log.e(TAG, "Invalid input: empty image lists");
            return null;
        }

        if (leftImages.size() != rightImages.size()) {
            Log.e(TAG, "Left and right image counts don't match: " + leftImages.size() + " vs " + rightImages.size());
            return null;
        }

        // Load pre-saved single camera calibrations
        SingleCameraCalibration.CameraCalibrationData leftCameraData =
                singleCameraCalibration.loadCalibrationData(leftCameraName);
        SingleCameraCalibration.CameraCalibrationData rightCameraData =
                singleCameraCalibration.loadCalibrationData(rightCameraName);

        if (leftCameraData == null || rightCameraData == null) {
            Log.e(TAG, "Failed to load single camera calibration data");
            Log.e(TAG, "Make sure to run single camera calibration first for both cameras");
            return null;
        }

        if (!leftCameraData.isValid() || !rightCameraData.isValid()) {
            Log.e(TAG, "Invalid single camera calibration data");
            return null;
        }

        Log.d(TAG, "Loaded single camera calibrations:");
        Log.d(TAG, "Left camera RMS: " + leftCameraData.rms);
        Log.d(TAG, "Right camera RMS: " + rightCameraData.rms);

        // Find corresponding chessboard corners in stereo image pairs
        List<Mat> leftImagePoints = new ArrayList<>();
        List<Mat> rightImagePoints = new ArrayList<>();
        List<Mat> objectPointsList = new ArrayList<>();

        // Generate 3D object points for chessboard
        Mat objectPoint = generateChessboardObjectPoints();

        Size imageSize = null;
        int validPairs = 0;

        // Process each stereo image pair
        for (int i = 0; i < leftImages.size(); i++) {
            Mat leftImage = leftImages.get(i);
            Mat rightImage = rightImages.get(i);

            if (leftImage == null || rightImage == null || leftImage.empty() || rightImage.empty()) {
                Log.w(TAG, "Skipping empty stereo pair " + (i + 1));
                continue;
            }

            if (imageSize == null) {
                imageSize = leftImage.size();
                Log.d(TAG, "Stereo image size: " + imageSize.width + "x" + imageSize.height);
            }

            // Convert to grayscale
            Mat leftGray = new Mat();
            Mat rightGray = new Mat();

            if (leftImage.channels() == 3) {
                Imgproc.cvtColor(leftImage, leftGray, Imgproc.COLOR_BGR2GRAY);
            } else if (leftImage.channels() == 4) {
                Imgproc.cvtColor(leftImage, leftGray, Imgproc.COLOR_BGRA2GRAY);
            } else {
                leftGray = leftImage.clone();
            }

            if (rightImage.channels() == 3) {
                Imgproc.cvtColor(rightImage, rightGray, Imgproc.COLOR_BGR2GRAY);
            } else if (rightImage.channels() == 4) {
                Imgproc.cvtColor(rightImage, rightGray, Imgproc.COLOR_BGRA2GRAY);
            } else {
                rightGray = rightImage.clone();
            }

            // Find chessboard corners in both images
            MatOfPoint2f leftCorners = new MatOfPoint2f();
            MatOfPoint2f rightCorners = new MatOfPoint2f();

            boolean leftFound = Calib3d.findChessboardCorners(leftGray, CHESSBOARD_SIZE, leftCorners);
            boolean rightFound = Calib3d.findChessboardCorners(rightGray, CHESSBOARD_SIZE, rightCorners);

            if (leftFound && rightFound) {
                // Refine corner positions for sub-pixel accuracy
                Imgproc.cornerSubPix(leftGray, leftCorners, new Size(11, 11), new Size(-1, -1),
                        new org.opencv.core.TermCriteria(
                                org.opencv.core.TermCriteria.EPS + org.opencv.core.TermCriteria.COUNT,
                                30, 0.1));

                Imgproc.cornerSubPix(rightGray, rightCorners, new Size(11, 11), new Size(-1, -1),
                        new org.opencv.core.TermCriteria(
                                org.opencv.core.TermCriteria.EPS + org.opencv.core.TermCriteria.COUNT,
                                30, 0.1));

                leftImagePoints.add(leftCorners);
                rightImagePoints.add(rightCorners);
                objectPointsList.add(objectPoint.clone());
                validPairs++;

                Log.d(TAG, "Found chessboard corners in stereo pair " + (i + 1) + " (" + validPairs + " total)");
            } else {
                Log.w(TAG, "Could not find chessboard corners in stereo pair " + (i + 1));
                if (!leftFound) Log.w(TAG, "  - Left image: no corners found");
                if (!rightFound) Log.w(TAG, "  - Right image: no corners found");
            }
        }

        if (validPairs < 4) {
            Log.e(TAG, "Not enough valid stereo pairs. Found: " + validPairs + ", need at least 10");
            return null;
        }

        // Create stereo calibration data object
        StereoCalibrationData stereoData = new StereoCalibrationData();
        stereoData.imageSize = imageSize;

        // Perform stereo calibration
        Log.d(TAG, "Performing stereo calibration with " + validPairs + " image pairs...");

        stereoData.rms = Calib3d.stereoCalibrate(
                objectPointsList,
                leftImagePoints,
                rightImagePoints,
                leftCameraData.cameraMatrix,
                leftCameraData.distortionCoeffs,
                rightCameraData.cameraMatrix,
                rightCameraData.distortionCoeffs,
                imageSize,
                stereoData.R,
                stereoData.T,
                stereoData.E,
                stereoData.F,
                Calib3d.CALIB_FIX_INTRINSIC
        );

        Log.d(TAG, "Stereo calibration completed");
        Log.d(TAG, "Stereo RMS reprojection error: " + stereoData.rms);

        // Log translation and rotation
        double[] translation = new double[3];
        stereoData.T.get(0, 0, translation);
        Log.d(TAG, String.format("Translation (mm): [%.2f, %.2f, %.2f]",
                translation[0], translation[1], translation[2]));

        // Compute rectification transforms
        Log.d(TAG, "Computing rectification transforms...");

        Calib3d.stereoRectify(
                leftCameraData.cameraMatrix,
                leftCameraData.distortionCoeffs,
                rightCameraData.cameraMatrix,
                rightCameraData.distortionCoeffs,
                imageSize,
                stereoData.R,
                stereoData.T,
                stereoData.R1,
                stereoData.R2,
                stereoData.P1,
                stereoData.P2,
                stereoData.Q,
                Calib3d.CALIB_ZERO_DISPARITY,
                -1,
                imageSize,
                null,
                null
        );

        // Log the Q matrix (important for 3D reconstruction)
        double[] qMatrix = new double[16];
        stereoData.Q.get(0, 0, qMatrix);
        Log.d(TAG, "Q Matrix (for 3D reconstruction):");
        Log.d(TAG, String.format("Q[0,0]=%.6f, Q[1,1]=%.6f", qMatrix[0], qMatrix[5]));
        Log.d(TAG, String.format("Q[0,3]=%.6f, Q[1,3]=%.6f", qMatrix[3], qMatrix[7]));
        Log.d(TAG, String.format("Q[2,3]=%.6f, Q[3,2]=%.6f", qMatrix[11], qMatrix[14]));

        // Save stereo calibration data
        if (saveStereoCalibrationData(stereoData)) {
            Log.d(TAG, "Stereo calibration data saved successfully");
        } else {
            Log.e(TAG, "Failed to save stereo calibration data");
        }

        Log.d(TAG, "Stereo calibration and rectification completed successfully");
        return stereoData;
    }

    /**
     * Load previously saved stereo calibration data from JSON
     *
     * @return StereoCalibrationData or null if loading failed
     */
    public StereoCalibrationData loadStereoCalibrationData() {
        try {
            String filename = "stereo_calibration.json";
            File file = new File(calibrationDataPath, filename);

            if (!file.exists()) {
                Log.e(TAG, "Stereo calibration file not found: " + filename);
                return null;
            }

            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder jsonBuilder = new StringBuilder();
            String line;

            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
            }
            reader.close();

            String jsonContent = jsonBuilder.toString();
            StereoCalibrationData data = parseStereoCalibrationDataFromJson(jsonContent);

            if (data != null && data.isValid()) {
                Log.d(TAG, "Loaded valid stereo calibration data");
                Log.d(TAG, "Stereo RMS: " + data.rms);
            } else {
                Log.e(TAG, "Loaded stereo calibration data contains invalid values");
                return null;
            }

            return data;

        } catch (Exception e) {
            Log.e(TAG, "Error loading stereo calibration data", e);
            return null;
        }
    }

    /**
     * Parse stereo calibration data from JSON string
     */
    private StereoCalibrationData parseStereoCalibrationDataFromJson(String jsonContent) {
        try {
            StereoCalibrationData data = new StereoCalibrationData();

            // Simple JSON parsing without external libraries
            // Extract arrays for each matrix
            String RStr = extractJsonArray(jsonContent, "R");
            double[] RArray = parseDoubleArray(RStr);

            String TStr = extractJsonArray(jsonContent, "T");
            double[] TArray = parseDoubleArray(TStr);

            String R1Str = extractJsonArray(jsonContent, "R1");
            double[] R1Array = parseDoubleArray(R1Str);

            String R2Str = extractJsonArray(jsonContent, "R2");
            double[] R2Array = parseDoubleArray(R2Str);

            String P1Str = extractJsonArray(jsonContent, "P1");
            double[] P1Array = parseDoubleArray(P1Str);

            String P2Str = extractJsonArray(jsonContent, "P2");
            double[] P2Array = parseDoubleArray(P2Str);

            String QStr = extractJsonArray(jsonContent, "Q");
            double[] QArray = parseDoubleArray(QStr);

            // Extract RMS value
            data.rms = extractJsonDouble(jsonContent, "rms");

            // Extract image size
            double width = extractJsonDouble(jsonContent, "width");
            double height = extractJsonDouble(jsonContent, "height");

            // Reconstruct Mat objects
            data.imageSize = new Size(width, height);

            data.R = new Mat(3, 3, CvType.CV_64F);
            data.T = new Mat(3, 1, CvType.CV_64F);
            data.R1 = new Mat(3, 3, CvType.CV_64F);
            data.R2 = new Mat(3, 3, CvType.CV_64F);
            data.P1 = new Mat(3, 4, CvType.CV_64F);
            data.P2 = new Mat(3, 4, CvType.CV_64F);
            data.Q = new Mat(4, 4, CvType.CV_64F);

            data.R.put(0, 0, RArray);
            data.T.put(0, 0, TArray);
            data.R1.put(0, 0, R1Array);
            data.R2.put(0, 0, R2Array);
            data.P1.put(0, 0, P1Array);
            data.P2.put(0, 0, P2Array);
            data.Q.put(0, 0, QArray);

            return data;

        } catch (Exception e) {
            Log.e(TAG, "Error parsing stereo calibration data from JSON", e);
            return null;
        }
    }

    /**
     * Extract JSON array as string
     */
    private String extractJsonArray(String jsonContent, String key) {
        String searchStr = "\"" + key + "\": [";
        int startIndex = jsonContent.indexOf(searchStr);
        if (startIndex == -1) return "";

        startIndex += searchStr.length();
        int endIndex = jsonContent.indexOf("]", startIndex);
        if (endIndex == -1) return "";

        return jsonContent.substring(startIndex, endIndex);
    }

    /**
     * Extract JSON double value
     */
    private double extractJsonDouble(String jsonContent, String key) {
        String searchStr = "\"" + key + "\": ";
        int startIndex = jsonContent.indexOf(searchStr);
        if (startIndex == -1) return 0.0;

        startIndex += searchStr.length();
        int endIndex = jsonContent.indexOf(",", startIndex);
        if (endIndex == -1) {
            endIndex = jsonContent.indexOf("\n", startIndex);
            if (endIndex == -1) {
                endIndex = jsonContent.indexOf("}", startIndex);
            }
        }

        String valueStr = jsonContent.substring(startIndex, endIndex).trim();
        return Double.parseDouble(valueStr);
    }

    /**
     * Parse comma-separated double array
     */
    private double[] parseDoubleArray(String arrayStr) {
        String[] parts = arrayStr.split(",");
        double[] result = new double[parts.length];

        for (int i = 0; i < parts.length; i++) {
            result[i] = Double.parseDouble(parts[i].trim());
        }

        return result;
    }

    /**
     * Check if stereo calibration data exists
     *
     * @return true if stereo calibration data file exists
     */
    public boolean hasStereoCalibrationData() {
        String filename = "stereo_calibration.json";
        File file = new File(calibrationDataPath, filename);
        return file.exists();
    }

    /**
     * Validate stereo chessboard detection in image pair (for testing)
     *
     * @param leftImage Left camera image
     * @param rightImage Right camera image
     * @return true if chessboard pattern is detected in both images
     */
    public boolean validateStereoChessboardDetection(Mat leftImage, Mat rightImage) {
        if (leftImage == null || rightImage == null || leftImage.empty() || rightImage.empty()) {
            Log.w(TAG, "Invalid input images for stereo validation");
            return false;
        }

        // Convert to grayscale
        Mat leftGray = new Mat();
        Mat rightGray = new Mat();

        if (leftImage.channels() == 3) {
            Imgproc.cvtColor(leftImage, leftGray, Imgproc.COLOR_BGR2GRAY);
        } else if (leftImage.channels() == 4) {
            Imgproc.cvtColor(leftImage, leftGray, Imgproc.COLOR_BGRA2GRAY);
        } else {
            leftGray = leftImage.clone();
        }

        if (rightImage.channels() == 3) {
            Imgproc.cvtColor(rightImage, rightGray, Imgproc.COLOR_BGR2GRAY);
        } else if (rightImage.channels() == 4) {
            Imgproc.cvtColor(rightImage, rightGray, Imgproc.COLOR_BGRA2GRAY);
        } else {
            rightGray = rightImage.clone();
        }

        // Find chessboard corners in both images
        MatOfPoint2f leftCorners = new MatOfPoint2f();
        MatOfPoint2f rightCorners = new MatOfPoint2f();

        boolean leftFound = Calib3d.findChessboardCorners(leftGray, CHESSBOARD_SIZE, leftCorners);
        boolean rightFound = Calib3d.findChessboardCorners(rightGray, CHESSBOARD_SIZE, rightCorners);

        if (leftFound && rightFound) {
            Log.d(TAG, "Stereo chessboard pattern detected successfully in both images");
            return true;
        } else {
            Log.w(TAG, "Stereo chessboard pattern detection failed:");
            if (!leftFound) Log.w(TAG, "  - Left image: no corners found");
            if (!rightFound) Log.w(TAG, "  - Right image: no corners found");
            return false;
        }
    }

    /**
     * Generate 3D object points for chessboard pattern
     */
    private Mat generateChessboardObjectPoints() {
        List<Point3> objectPointsList = new ArrayList<>();

        for (int i = 0; i < CHESSBOARD_HEIGHT; i++) {
            for (int j = 0; j < CHESSBOARD_WIDTH; j++) {
                objectPointsList.add(new Point3(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
            }
        }

        MatOfPoint3f objectPoints = new MatOfPoint3f();
        objectPoints.fromList(objectPointsList);
        return objectPoints;
    }

    /**
     * Save stereo calibration data to JSON file
     */
    private boolean saveStereoCalibrationData(StereoCalibrationData data) {
        try {
            String filename = "stereo_calibration.json";
            File file = new File(calibrationDataPath, filename);

            // Validate data before saving
            if (!data.isValid()) {
                Log.e(TAG, "Cannot save stereo calibration data with invalid values");
                return false;
            }

            // Convert Mat objects to arrays for JSON serialization
            double[] RArray = new double[(int) data.R.total() * data.R.channels()];
            data.R.get(0, 0, RArray);

            double[] TArray = new double[(int) data.T.total() * data.T.channels()];
            data.T.get(0, 0, TArray);

            double[] R1Array = new double[(int) data.R1.total() * data.R1.channels()];
            data.R1.get(0, 0, R1Array);

            double[] R2Array = new double[(int) data.R2.total() * data.R2.channels()];
            data.R2.get(0, 0, R2Array);

            double[] P1Array = new double[(int) data.P1.total() * data.P1.channels()];
            data.P1.get(0, 0, P1Array);

            double[] P2Array = new double[(int) data.P2.total() * data.P2.channels()];
            data.P2.get(0, 0, P2Array);

            double[] QArray = new double[(int) data.Q.total() * data.Q.channels()];
            data.Q.get(0, 0, QArray);

            // Create JSON content manually
            StringBuilder jsonBuilder = new StringBuilder();
            jsonBuilder.append("{\n");

            // Rotation matrix R
            jsonBuilder.append("  \"R\": [");
            for (int i = 0; i < RArray.length; i++) {
                jsonBuilder.append(String.format("%.8f", RArray[i]));
                if (i < RArray.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Translation vector T
            jsonBuilder.append("  \"T\": [");
            for (int i = 0; i < TArray.length; i++) {
                jsonBuilder.append(String.format("%.8f", TArray[i]));
                if (i < TArray.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Rectification matrix R1
            jsonBuilder.append("  \"R1\": [");
            for (int i = 0; i < R1Array.length; i++) {
                jsonBuilder.append(String.format("%.8f", R1Array[i]));
                if (i < R1Array.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Rectification matrix R2
            jsonBuilder.append("  \"R2\": [");
            for (int i = 0; i < R2Array.length; i++) {
                jsonBuilder.append(String.format("%.8f", R2Array[i]));
                if (i < R2Array.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Projection matrix P1
            jsonBuilder.append("  \"P1\": [");
            for (int i = 0; i < P1Array.length; i++) {
                jsonBuilder.append(String.format("%.8f", P1Array[i]));
                if (i < P1Array.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Projection matrix P2
            jsonBuilder.append("  \"P2\": [");
            for (int i = 0; i < P2Array.length; i++) {
                jsonBuilder.append(String.format("%.8f", P2Array[i]));
                if (i < P2Array.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // Disparity-to-depth mapping matrix Q
            jsonBuilder.append("  \"Q\": [");
            for (int i = 0; i < QArray.length; i++) {
                jsonBuilder.append(String.format("%.8f", QArray[i]));
                if (i < QArray.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            // RMS reprojection error
            jsonBuilder.append("  \"rms\": ").append(String.format("%.8f", data.rms)).append(",\n");

            // Image size
            jsonBuilder.append("  \"imageSize\": {\n");
            jsonBuilder.append("    \"width\": ").append(data.imageSize.width).append(",\n");
            jsonBuilder.append("    \"height\": ").append(data.imageSize.height).append("\n");
            jsonBuilder.append("  }\n");
            jsonBuilder.append("}");

            // Write JSON to file
            BufferedWriter writer = new BufferedWriter(new FileWriter(file));
            writer.write(jsonBuilder.toString());
            writer.close();

            Log.d(TAG, "Saved stereo calibration data in JSON format");
            return true;

        } catch (Exception e) {
            Log.e(TAG, "Error saving stereo calibration data", e);
            return false;
        }
    }

    /**
     * Get chessboard configuration (should match SingleCameraCalibration)
     */
    public static Size getChessboardSize() {
        return CHESSBOARD_SIZE;
    }

    public static float getSquareSize() {
        return SQUARE_SIZE;
    }

    /**
     * Get baseline distance between cameras from stereo calibration
     *
     * @param stereoData Stereo calibration data
     * @return Baseline distance in same units as SQUARE_SIZE
     */
    public static double getBaseline(StereoCalibrationData stereoData) {
        if (stereoData == null || !stereoData.isValid()) {
            return 0.0;
        }

        // Baseline is the magnitude of the translation vector
        double[] translation = new double[3];
        stereoData.T.get(0, 0, translation);

        return Math.sqrt(translation[0] * translation[0] +
                translation[1] * translation[1] +
                translation[2] * translation[2]);
    }
}