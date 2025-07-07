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
import java.util.ArrayList;
import java.util.List;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
import java.io.Serializable;

/**
 * Single Camera Calibration for FTC Stereo Vision System
 * Step 1: Calibrates individual cameras using chessboard pattern
 * Updated to save/load calibration data in JSON format
 */
public class SingleCameraCalibration {
    private static final String TAG = "SingleCameraCalibration";

    // Chessboard pattern size (internal corners)
    private static final int CHESSBOARD_WIDTH = 9;
    private static final int CHESSBOARD_HEIGHT = 6;
    private static final Size CHESSBOARD_SIZE = new Size(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT);

    // Square size in real world units (e.g., 25mm)
    private static final float SQUARE_SIZE = 25.0f;

    // Minimum number of valid images required for calibration
    private static final int MIN_CALIBRATION_IMAGES = 10;

    /**
     * Camera calibration data storage class
     */
    public static class CameraCalibrationData implements Serializable {
        public Mat cameraMatrix;
        public Mat distortionCoeffs;
        public double rms;
        public Size imageSize;

        public CameraCalibrationData() {
            cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            distortionCoeffs = new Mat(1, 5, CvType.CV_64F);
        }

        public boolean isValid() {
            return cameraMatrix != null && !cameraMatrix.empty() &&
                    distortionCoeffs != null && !distortionCoeffs.empty() &&
                    !Double.isNaN(rms) && rms > 0;
        }

        public boolean hasValidValues() {
            // Check camera matrix for NaN values
            double[] cameraData = new double[9];
            cameraMatrix.get(0, 0, cameraData);
            for (double val : cameraData) {
                if (Double.isNaN(val) || Double.isInfinite(val)) {
                    return false;
                }
            }

            // Check distortion coefficients for NaN values
            double[] distortionData = new double[5];
            distortionCoeffs.get(0, 0, distortionData);
            for (double val : distortionData) {
                if (Double.isNaN(val) || Double.isInfinite(val)) {
                    return false;
                }
            }

            return !Double.isNaN(rms) && !Double.isInfinite(rms);
        }
    }

    // File path for saving calibration data
    private String calibrationDataPath;

    public SingleCameraCalibration(String dataPath) {
        this.calibrationDataPath = dataPath;

        // Create directory if it doesn't exist
        File dir = new File(dataPath);
        if (!dir.exists()) {
            dir.mkdirs();
        }
    }

    /**
     * Calibrate a single camera using chessboard pattern
     *
     * @param calibrationImages List of images containing chessboard pattern
     * @param cameraName Name identifier for this camera (e.g., "left", "right")
     * @return CameraCalibrationData containing calibration results
     */
    public CameraCalibrationData calibrateCamera(List<Mat> calibrationImages, String cameraName) {
        Log.d(TAG, "Starting camera calibration for: " + cameraName);

        if (calibrationImages == null || calibrationImages.isEmpty()) {
            Log.e(TAG, "No calibration images provided");
            return null;
        }

        List<Mat> imagePoints = new ArrayList<>();
        List<Mat> objectPointsList = new ArrayList<>();

        // Generate 3D object points for chessboard
        Mat objectPoints = generateChessboardObjectPoints();

        Size imageSize = null;
        int validImages = 0;

        // Process each calibration image
        for (int i = 0; i < calibrationImages.size(); i++) {
            Mat image = calibrationImages.get(i);

            if (image == null || image.empty()) {
                Log.w(TAG, "Skipping empty image " + (i + 1));
                continue;
            }

            if (imageSize == null) {
                imageSize = image.size();
                Log.d(TAG, "Image size: " + imageSize.width + "x" + imageSize.height);
            }

            // Convert to grayscale if needed
            Mat grayImage = new Mat();
            if (image.channels() == 3) {
                Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
            } else if (image.channels() == 4) {
                Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGRA2GRAY);
            } else {
                grayImage = image.clone();
            }

            // Find chessboard corners
            MatOfPoint2f corners = new MatOfPoint2f();
            boolean found = Calib3d.findChessboardCorners(grayImage, CHESSBOARD_SIZE, corners,
                    Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                // Validate that we have the expected number of corners
                if (corners.total() != CHESSBOARD_WIDTH * CHESSBOARD_HEIGHT) {
                    Log.w(TAG, "Incorrect number of corners found in image " + (i + 1) +
                            ". Expected: " + (CHESSBOARD_WIDTH * CHESSBOARD_HEIGHT) +
                            ", Found: " + corners.total());
                    continue;
                }

                // Refine corner positions for sub-pixel accuracy
                Imgproc.cornerSubPix(grayImage, corners, new Size(11, 11), new Size(-1, -1),
                        new org.opencv.core.TermCriteria(
                                org.opencv.core.TermCriteria.EPS + org.opencv.core.TermCriteria.COUNT,
                                30, 0.1));

                imagePoints.add(corners);
                objectPointsList.add(objectPoints.clone());
                validImages++;

                Log.d(TAG, "Found chessboard corners in image " + (i + 1) + " (" + validImages + " total)");
            } else {
                Log.w(TAG, "Could not find chessboard corners in image " + (i + 1));
            }
        }

        if (validImages < MIN_CALIBRATION_IMAGES) {
            Log.e(TAG, "Not enough valid calibration images. Found: " + validImages +
                    ", need at least " + MIN_CALIBRATION_IMAGES);
            return null;
        }

        // Create calibration data object
        CameraCalibrationData calibrationData = new CameraCalibrationData();
        calibrationData.imageSize = imageSize;

        // Initialize camera matrix with reasonable initial values
        calibrationData.cameraMatrix.put(0, 0, new double[]{
                imageSize.width, 0, imageSize.width / 2,
                0, imageSize.width, imageSize.height / 2,
                0, 0, 1
        });

        // Initialize distortion coefficients to zero
        calibrationData.distortionCoeffs.put(0, 0, new double[]{0, 0, 0, 0, 0});

        // Lists to store rotation and translation vectors (not used but required by OpenCV)
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();

        // Perform camera calibration with more permissive flags
        Log.d(TAG, "Performing camera calibration with " + validImages + " valid images...");

        try {
            // Use less restrictive calibration flags
            int calibrationFlags = Calib3d.CALIB_FIX_PRINCIPAL_POINT +
                    Calib3d.CALIB_ZERO_TANGENT_DIST;

            calibrationData.rms = Calib3d.calibrateCamera(
                    objectPointsList,
                    imagePoints,
                    imageSize,
                    calibrationData.cameraMatrix,
                    calibrationData.distortionCoeffs,
                    rvecs,
                    tvecs,
                    calibrationFlags
            );

            // Validate the calibration results
            if (!calibrationData.hasValidValues()) {
                Log.e(TAG, "Calibration produced invalid results (NaN/Inf values)");

                // Try again with different flags
                Log.d(TAG, "Retrying calibration with default flags...");
                calibrationData.cameraMatrix.put(0, 0, new double[]{
                        imageSize.width, 0, imageSize.width / 2,
                        0, imageSize.width, imageSize.height / 2,
                        0, 0, 1
                });
                calibrationData.distortionCoeffs.put(0, 0, new double[]{0, 0, 0, 0, 0});

                calibrationData.rms = Calib3d.calibrateCamera(
                        objectPointsList,
                        imagePoints,
                        imageSize,
                        calibrationData.cameraMatrix,
                        calibrationData.distortionCoeffs,
                        rvecs,
                        tvecs,
                        0  // No flags - let OpenCV estimate everything
                );
            }

            if (!calibrationData.hasValidValues()) {
                Log.e(TAG, "Calibration still producing invalid results after retry");
                return null;
            }

            Log.d(TAG, "Camera calibration completed for " + cameraName);
            Log.d(TAG, "RMS reprojection error: " + calibrationData.rms);

            // Log camera matrix
            double[] cameraMatrixArray = new double[9];
            calibrationData.cameraMatrix.get(0, 0, cameraMatrixArray);
            Log.d(TAG, "Camera Matrix:");
            Log.d(TAG, String.format("fx=%.2f, fy=%.2f", cameraMatrixArray[0], cameraMatrixArray[4]));
            Log.d(TAG, String.format("cx=%.2f, cy=%.2f", cameraMatrixArray[2], cameraMatrixArray[5]));

            // Log distortion coefficients
            double[] distortionArray = new double[5];
            calibrationData.distortionCoeffs.get(0, 0, distortionArray);
            Log.d(TAG, "Distortion coefficients:");
            Log.d(TAG, String.format("k1=%.6f, k2=%.6f, p1=%.6f, p2=%.6f, k3=%.6f",
                    distortionArray[0], distortionArray[1], distortionArray[2],
                    distortionArray[3], distortionArray[4]));

            // Validate RMS error
            if (calibrationData.rms > 1.0) {
                Log.w(TAG, "High RMS reprojection error: " + calibrationData.rms +
                        ". Consider improving calibration images.");
            }

        } catch (Exception e) {
            Log.e(TAG, "Error during camera calibration", e);
            return null;
        }

        // Save calibration data
        if (saveCalibrationData(calibrationData, cameraName)) {
            Log.d(TAG, "Calibration data saved successfully for " + cameraName);
        } else {
            Log.e(TAG, "Failed to save calibration data for " + cameraName);
        }

        return calibrationData;
    }

    /**
     * Load previously saved camera calibration data from JSON
     *
     * @param cameraName Name identifier for the camera
     * @return CameraCalibrationData or null if loading failed
     */
    public CameraCalibrationData loadCalibrationData(String cameraName) {
        try {
            String filename = cameraName + "_camera_calibration.json";
            File file = new File(calibrationDataPath, filename);

            if (!file.exists()) {
                Log.e(TAG, "Calibration file not found: " + filename);
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
            CameraCalibrationData data = parseCalibrationDataFromJson(jsonContent);

            if (data != null && data.hasValidValues()) {
                Log.d(TAG, "Loaded valid calibration data for " + cameraName);
                Log.d(TAG, "RMS: " + data.rms);
            } else {
                Log.e(TAG, "Loaded calibration data contains invalid values for " + cameraName);
                return null;
            }

            return data;

        } catch (Exception e) {
            Log.e(TAG, "Error loading calibration data for " + cameraName, e);
            return null;
        }
    }

    /**
     * Check if calibration data exists for a camera
     *
     * @param cameraName Name identifier for the camera
     * @return true if calibration data file exists
     */
    public boolean hasCalibrationData(String cameraName) {
        String filename = cameraName + "_camera_calibration.json";
        File file = new File(calibrationDataPath, filename);
        return file.exists();
    }

    /**
     * Generate 3D object points for chessboard pattern
     * Fixed to use proper coordinate system
     */
    private Mat generateChessboardObjectPoints() {
        List<Point3> objectPointsList = new ArrayList<>();

        // Generate points in proper order (row-major)
        for (int i = 0; i < CHESSBOARD_HEIGHT; i++) {
            for (int j = 0; j < CHESSBOARD_WIDTH; j++) {
                // Use millimeter units and proper coordinate system
                objectPointsList.add(new Point3(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0));
            }
        }

        MatOfPoint3f objectPoints = new MatOfPoint3f();
        objectPoints.fromList(objectPointsList);
        return objectPoints;
    }

    /**
     * Save camera calibration data to JSON file
     */
    private boolean saveCalibrationData(CameraCalibrationData data, String cameraName) {
        try {
            String filename = cameraName + "_camera_calibration.json";
            File file = new File(calibrationDataPath, filename);

            // Validate data before saving
            if (!data.hasValidValues()) {
                Log.e(TAG, "Cannot save calibration data with invalid values for " + cameraName);
                return false;
            }

            // Convert Mat objects to arrays for JSON serialization
            double[] cameraMatrixArray = new double[(int) data.cameraMatrix.total() * data.cameraMatrix.channels()];
            data.cameraMatrix.get(0, 0, cameraMatrixArray);

            double[] distortionArray = new double[(int) data.distortionCoeffs.total() * data.distortionCoeffs.channels()];
            data.distortionCoeffs.get(0, 0, distortionArray);

            // Create JSON content manually
            StringBuilder jsonBuilder = new StringBuilder();
            jsonBuilder.append("{\n");
            jsonBuilder.append("  \"cameraMatrix\": [");
            for (int i = 0; i < cameraMatrixArray.length; i++) {
                jsonBuilder.append(String.format("%.8f", cameraMatrixArray[i]));
                if (i < cameraMatrixArray.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            jsonBuilder.append("  \"distortionCoeffs\": [");
            for (int i = 0; i < distortionArray.length; i++) {
                jsonBuilder.append(String.format("%.8f", distortionArray[i]));
                if (i < distortionArray.length - 1) {
                    jsonBuilder.append(", ");
                }
            }
            jsonBuilder.append("],\n");

            jsonBuilder.append("  \"rms\": ").append(String.format("%.8f", data.rms)).append(",\n");
            jsonBuilder.append("  \"imageSize\": {\n");
            jsonBuilder.append("    \"width\": ").append(data.imageSize.width).append(",\n");
            jsonBuilder.append("    \"height\": ").append(data.imageSize.height).append("\n");
            jsonBuilder.append("  }\n");
            jsonBuilder.append("}");

            // Write JSON to file
            BufferedWriter writer = new BufferedWriter(new FileWriter(file));
            writer.write(jsonBuilder.toString());
            writer.close();

            Log.d(TAG, "Saved calibration data for " + cameraName + " in JSON format");
            return true;

        } catch (Exception e) {
            Log.e(TAG, "Error saving calibration data for " + cameraName, e);
            return false;
        }
    }

    /**
     * Parse calibration data from JSON string
     */
    private CameraCalibrationData parseCalibrationDataFromJson(String jsonContent) {
        try {
            CameraCalibrationData data = new CameraCalibrationData();

            // Simple JSON parsing without external libraries
            // Extract camera matrix array
            String cameraMatrixStr = extractJsonArray(jsonContent, "cameraMatrix");
            double[] cameraMatrixArray = parseDoubleArray(cameraMatrixStr);

            // Extract distortion coefficients array
            String distortionStr = extractJsonArray(jsonContent, "distortionCoeffs");
            double[] distortionArray = parseDoubleArray(distortionStr);

            // Extract RMS value
            data.rms = extractJsonDouble(jsonContent, "rms");

            // Extract image size
            double width = extractJsonDouble(jsonContent, "width");
            double height = extractJsonDouble(jsonContent, "height");

            // Reconstruct Mat objects
            data.imageSize = new Size(width, height);
            data.cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            data.distortionCoeffs = new Mat(1, 5, CvType.CV_64F);

            data.cameraMatrix.put(0, 0, cameraMatrixArray);
            data.distortionCoeffs.put(0, 0, distortionArray);

            return data;

        } catch (Exception e) {
            Log.e(TAG, "Error parsing calibration data from JSON", e);
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
     * Validate chessboard detection in a single image (for testing)
     *
     * @param image Input image
     * @return true if chessboard pattern is detected
     */
    public boolean validateChessboardDetection(Mat image) {
        if (image == null || image.empty()) {
            return false;
        }

        // Convert to grayscale if needed
        Mat grayImage = new Mat();
        if (image.channels() == 3) {
            Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
        } else if (image.channels() == 4) {
            Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGRA2GRAY);
        } else {
            grayImage = image.clone();
        }

        // Find chessboard corners with improved flags
        MatOfPoint2f corners = new MatOfPoint2f();
        boolean found = Calib3d.findChessboardCorners(grayImage, CHESSBOARD_SIZE, corners,
                Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE);

        if (found && corners.total() == CHESSBOARD_WIDTH * CHESSBOARD_HEIGHT) {
            Log.d(TAG, "Chessboard pattern detected successfully with " + corners.total() + " corners");
        } else {
            Log.w(TAG, "Chessboard pattern not detected or incorrect number of corners");
        }

        return found && corners.total() == CHESSBOARD_WIDTH * CHESSBOARD_HEIGHT;
    }

    /**
     * Get chessboard configuration
     */
    public static Size getChessboardSize() {
        return CHESSBOARD_SIZE;
    }

    public static float getSquareSize() {
        return SQUARE_SIZE;
    }
}