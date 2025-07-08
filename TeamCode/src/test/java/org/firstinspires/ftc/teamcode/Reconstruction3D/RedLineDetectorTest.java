package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.junit.Test;
import org.junit.Before;
import org.junit.After;
import org.junit.BeforeClass;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import static org.junit.Assert.*;

import java.io.File;
import java.net.URL;
import java.util.List;

public class RedLineDetectorTest {

    private RedLineDetector detector;
    private static final String TEST_IMAGES_PATH = "test/resources/RedlineDetector/";
    private static final String LEFT_IMAGE_NAME = "test_left_1.jpg";
    private static final String RIGHT_IMAGE_NAME = "test_right_1.jpg";

    @BeforeClass
    public static void setupOpenCV() {
        // Load OpenCV native library
        try {
            // Try to load OpenCV using the standard approach
//            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            System.load("/Users/hao.wu/opencv_macos_install/share/java/opencv4/libopencv_java4110.dylib"); // or .so or .dll
            System.out.println("OpenCV library loaded successfully");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Warning: Failed to load OpenCV library: " + e.getMessage());
            System.err.println("OpenCV-dependent tests will be skipped");
            System.err.println("To fix this, ensure OpenCV is installed and add to build.gradle:");
            System.err.println("  implementation 'org.opencv:opencv-java:4.5.4'");
            // Don't throw exception - let tests decide if they can run without OpenCV
        }
    }

    @Before
    public void setUp() {
        detector = new RedLineDetector();
    }

    @After
    public void tearDown() {
        if (detector != null) {
            detector.release();
        }
    }

    @Test
    public void testRedLineDetection() {
        // Load test images
        Mat leftFrame = loadTestImage(LEFT_IMAGE_NAME);
        Mat rightFrame = loadTestImage(RIGHT_IMAGE_NAME);

        assertNotNull("Failed to load left test image", leftFrame);
        assertNotNull("Failed to load right test image", rightFrame);
        assertFalse("Left image is empty", leftFrame.empty());
        assertFalse("Right image is empty", rightFrame.empty());

        // Detect red lines
        RedLineDetector.DetectionResult result = detector.detectRedLines(leftFrame, rightFrame);

        // Verify results
        assertNotNull("Detection result should not be null", result);
        assertNotNull("Left lines list should not be null", result.leftLines);
        assertNotNull("Right lines list should not be null", result.rightLines);

        // Print detection statistics
        System.out.println("=== Red Line Detection Results ===");
        System.out.println("Left camera lines detected: " + result.leftLines.size());
        System.out.println("Right camera lines detected: " + result.rightLines.size());
        System.out.println("Total lines detected: " + result.getTotalDetections());

        // Print individual line details
        printLineDetails("LEFT", result.leftLines);
        printLineDetails("RIGHT", result.rightLines);

        // Test filtering for floor lines
        List<RedLineDetector.LineSegment> leftFloorLines =
                detector.filterFloorLines(result.leftLines, leftFrame.rows());
        List<RedLineDetector.LineSegment> rightFloorLines =
                detector.filterFloorLines(result.rightLines, rightFrame.rows());

        System.out.println("\n=== Floor Line Filtering Results ===");
        System.out.println("Left floor lines: " + leftFloorLines.size());
        System.out.println("Right floor lines: " + rightFloorLines.size());

        // Create visualization images
        Mat leftVisualization = leftFrame.clone();
        Mat rightVisualization = rightFrame.clone();

        // Draw detected lines
        detector.drawDetectedLines(leftVisualization, leftFloorLines, new Scalar(0, 0, 255));
        detector.drawDetectedLines(rightVisualization, rightFloorLines, new Scalar(0, 0, 255));

        // Save visualization images
        saveVisualizationImage(leftVisualization, "left_detection_result.jpg");
        saveVisualizationImage(rightVisualization, "right_detection_result.jpg");

        // Clean up
        leftFrame.release();
        rightFrame.release();
        leftVisualization.release();
        rightVisualization.release();
    }

    @Test
    public void testImageLoadingOnly() {
        // Test just image loading functionality
        Mat leftFrame = loadTestImage(LEFT_IMAGE_NAME);
        Mat rightFrame = loadTestImage(RIGHT_IMAGE_NAME);

        if (leftFrame != null && !leftFrame.empty()) {
            System.out.println("Left image loaded successfully:");
            System.out.println("  Size: " + leftFrame.cols() + "x" + leftFrame.rows());
            System.out.println("  Channels: " + leftFrame.channels());
            System.out.println("  Type: " + leftFrame.type());
        }

        if (rightFrame != null && !rightFrame.empty()) {
            System.out.println("Right image loaded successfully:");
            System.out.println("  Size: " + rightFrame.cols() + "x" + rightFrame.rows());
            System.out.println("  Channels: " + rightFrame.channels());
            System.out.println("  Type: " + rightFrame.type());
        }

        // Clean up
        if (leftFrame != null) leftFrame.release();
        if (rightFrame != null) rightFrame.release();
    }

    @Test
    public void testDetectorInitialization() {
        // Test that detector initializes properly
        assertNotNull("Detector should be initialized", detector);

        // Test detection result structure
        RedLineDetector.DetectionResult result = new RedLineDetector.DetectionResult();
        assertNotNull("Left lines list should be initialized", result.leftLines);
        assertNotNull("Right lines list should be initialized", result.rightLines);
        assertFalse("Empty result should have no detections", result.hasDetections());
        assertEquals("Empty result should have 0 total detections", 0, result.getTotalDetections());
    }

    /**
     * Load test image from resources
     */
    private Mat loadTestImage(String imageName) {
        try {
            // Try to load from classpath first
            URL resourceUrl = getClass().getClassLoader().getResource(TEST_IMAGES_PATH + imageName);
            if (resourceUrl != null) {
                String imagePath = resourceUrl.getPath();
                Mat image = Imgcodecs.imread(imagePath);
                if (!image.empty()) {
                    return image;
                }
            }

            // Try to load from file system
            String[] possiblePaths = {
                    TEST_IMAGES_PATH + imageName,
                    "src/" + TEST_IMAGES_PATH + imageName,
                    "src/test/resources/RedlineDetector/" + imageName,
                    "test/resources/RedlineDetector/" + imageName
            };

            for (String path : possiblePaths) {
                File file = new File(path);
                if (file.exists()) {
                    Mat image = Imgcodecs.imread(file.getAbsolutePath());
                    if (!image.empty()) {
                        System.out.println("Successfully loaded image from: " + path);
                        return image;
                    }
                }
            }

            System.err.println("Failed to load image: " + imageName);
            System.err.println("Searched paths:");
            for (String path : possiblePaths) {
                System.err.println("  " + path + " (exists: " + new File(path).exists() + ")");
            }

            return null;

        } catch (Exception e) {
            System.err.println("Error loading image " + imageName + ": " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Save visualization image
     */
    private void saveVisualizationImage(Mat image, String filename) {
        try {
            String outputPath = "test_output/" + filename;

            // Create output directory if it doesn't exist
            File outputDir = new File("test_output");
            if (!outputDir.exists()) {
                outputDir.mkdirs();
            }

            boolean success = Imgcodecs.imwrite(outputPath, image);
            if (success) {
                System.out.println("Saved visualization image: " + outputPath);
            } else {
                System.err.println("Failed to save visualization image: " + outputPath);
            }
        } catch (Exception e) {
            System.err.println("Error saving visualization image: " + e.getMessage());
        }
    }

    /**
     * Print detailed information about detected lines
     */
    private void printLineDetails(String cameraId, List<RedLineDetector.LineSegment> lines) {
        System.out.println("\n--- " + cameraId + " Camera Lines ---");
        for (int i = 0; i < lines.size(); i++) {
            RedLineDetector.LineSegment line = lines.get(i);
            Point center = line.getCenter();
            double length = line.getLength();
            double angle = Math.toDegrees(line.getAngle());

            System.out.printf("Line %d: Start(%.1f,%.1f) End(%.1f,%.1f) Center(%.1f,%.1f) Length=%.1f Angle=%.1f°%n",
                    i + 1, line.start.x, line.start.y, line.end.x, line.end.y,
                    center.x, center.y, length, angle);
        }
    }

    /**
     * Manual test runner - useful for debugging
     */
    public static void main(String[] args) {
        try {
            // Setup OpenCV
            setupOpenCV();

            // Create and run test
            RedLineDetectorTest test = new RedLineDetectorTest();
            test.setUp();

            System.out.println("Running Red Line Detection Test...");
            test.testImageLoadingOnly();
            test.testRedLineDetection();

            test.tearDown();
            System.out.println("Test completed successfully!");

        } catch (Exception e) {
            System.err.println("Test failed: " + e.getMessage());
            e.printStackTrace();
        }
    }
}

// Additional utility class for creating test images if needed
class TestImageCreator {

    /**
     * Create a synthetic test image with red lines for testing
     */
    public static Mat createSyntheticTestImage(int width, int height, boolean withRedLines) {
        Mat image = new Mat(height, width, CvType.CV_8UC3, new Scalar(200, 200, 200));

        if (withRedLines) {
            // Add some red horizontal lines
            Imgproc.line(image, new Point(50, height * 0.7), new Point(width - 50, height * 0.7),
                    new Scalar(0, 0, 255), 5);
            Imgproc.line(image, new Point(100, height * 0.8), new Point(width - 100, height * 0.8),
                    new Scalar(0, 0, 255), 5);

            // Add some noise
            Mat noise = new Mat();
            Core.randn(noise, 0, 20);
            Core.add(image, noise, image);
            noise.release();
        }

        return image;
    }

    /**
     * Save synthetic test images
     */
    public static void createAndSaveTestImages() {
        try {
            // Create output directory
            File dir = new File("test/resources/RedlineDetector/");
            if (!dir.exists()) {
                dir.mkdirs();
            }

            // Create synthetic test images
            Mat leftImage = createSyntheticTestImage(640, 480, true);
            Mat rightImage = createSyntheticTestImage(640, 480, true);

            // Save images
            Imgcodecs.imwrite("test/resources/RedlineDetector/test_left_1.jpg", leftImage);
            Imgcodecs.imwrite("test/resources/RedlineDetector/test_right_1.jpg", rightImage);

            System.out.println("Created synthetic test images");

            leftImage.release();
            rightImage.release();

        } catch (Exception e) {
            System.err.println("Error creating test images: " + e.getMessage());
        }
    }
}