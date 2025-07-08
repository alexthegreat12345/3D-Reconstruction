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
        detector.setDebugMode(true); // Enable debug mode for testing
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
        System.out.println("Both cameras detected: " + result.hasBothCameraDetections());

        // Print individual line details
        printLineDetails("LEFT", result.leftLines);
        printLineDetails("RIGHT", result.rightLines);

        // Test the new getLinePoints method if we have detections
        if (result.hasDetections()) {
            testLinePointsGeneration(result);
        }

        // Create visualization images
        Mat leftVisualization = leftFrame.clone();
        Mat rightVisualization = rightFrame.clone();

        // Draw detected lines
        detector.drawDetectedLines(leftVisualization, result.leftLines, new Scalar(0, 0, 255));
        detector.drawDetectedLines(rightVisualization, result.rightLines, new Scalar(0, 0, 255));

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
    public void testColorMaskGeneration() {
        // Test the new getColorMask method
        Mat leftFrame = loadTestImage(LEFT_IMAGE_NAME);
        Mat rightFrame = loadTestImage(RIGHT_IMAGE_NAME);

        if (leftFrame != null && !leftFrame.empty()) {
            Mat leftMask = detector.getColorMask(leftFrame);
            assertNotNull("Left color mask should not be null", leftMask);
            assertFalse("Left color mask should not be empty", leftMask.empty());

            // Check mask properties
            assertEquals("Mask should be single channel", 1, leftMask.channels());
            assertEquals("Mask should be 8-bit", CvType.CV_8UC1, leftMask.type());

            // Count non-zero pixels
            int nonZeroPixels = Core.countNonZero(leftMask);
            System.out.println("Left mask non-zero pixels: " + nonZeroPixels);

            // Save mask for visualization
            saveVisualizationImage(leftMask, "left_color_mask.jpg");

            leftMask.release();
        }

        if (rightFrame != null && !rightFrame.empty()) {
            Mat rightMask = detector.getColorMask(rightFrame);
            assertNotNull("Right color mask should not be null", rightMask);
            assertFalse("Right color mask should not be empty", rightMask.empty());

            int nonZeroPixels = Core.countNonZero(rightMask);
            System.out.println("Right mask non-zero pixels: " + nonZeroPixels);

            saveVisualizationImage(rightMask, "right_color_mask.jpg");

            rightMask.release();
        }

        // Clean up
        if (leftFrame != null) leftFrame.release();
        if (rightFrame != null) rightFrame.release();
    }

    @Test
    public void testDebugMode() {
        // Test debug mode functionality
        detector.setDebugMode(false);

        Mat testFrame = loadTestImage(LEFT_IMAGE_NAME);
        if (testFrame != null && !testFrame.empty()) {
            System.out.println("Testing with debug mode OFF:");
            RedLineDetector.DetectionResult result = detector.detectRedLines(testFrame, testFrame);

            // Re-enable debug mode
            detector.setDebugMode(true);
            System.out.println("Testing with debug mode ON:");
            result = detector.detectRedLines(testFrame, testFrame);

            testFrame.release();
        }
    }

    @Test
    public void testLinePointsGeneration() {
        // Test getLinePoints method with synthetic line
        Point start = new Point(100, 200);
        Point end = new Point(500, 300);
        RedLineDetector.LineSegment testLine = new RedLineDetector.LineSegment(start, end, "TEST");

        // Test different numbers of points
        int[] pointCounts = {5, 10, 20, 50};

        for (int numPoints : pointCounts) {
            List<Point> points = detector.getLinePoints(testLine, numPoints);

            assertEquals("Should generate correct number of points", numPoints, points.size());

            // Verify first and last points match line endpoints
            if (numPoints > 0) {
                Point firstPoint = points.get(0);
                Point lastPoint = points.get(points.size() - 1);

                assertEquals("First point should match start", start.x, firstPoint.x, 0.1);
                assertEquals("First point should match start", start.y, firstPoint.y, 0.1);
                assertEquals("Last point should match end", end.x, lastPoint.x, 0.1);
                assertEquals("Last point should match end", end.y, lastPoint.y, 0.1);
            }

            System.out.println("Generated " + numPoints + " points along test line");
        }

        // Test with null line
        List<Point> nullResult = detector.getLinePoints(null, 10);
        assertTrue("Null line should return empty list", nullResult.isEmpty());
    }

    private void testLinePointsGeneration(RedLineDetector.DetectionResult result) {
        System.out.println("\n=== Testing Line Points Generation ===");

        // Test with left lines
        for (int i = 0; i < result.leftLines.size(); i++) {
            RedLineDetector.LineSegment line = result.leftLines.get(i);
            List<Point> points = detector.getLinePoints(line, 20);

            System.out.println("Left line " + i + " generated " + points.size() + " points");

            // Print first few points
            for (int j = 0; j < Math.min(5, points.size()); j++) {
                Point p = points.get(j);
                System.out.printf("  Point %d: (%.1f, %.1f)%n", j, p.x, p.y);
            }
        }

        // Test with right lines
        for (int i = 0; i < result.rightLines.size(); i++) {
            RedLineDetector.LineSegment line = result.rightLines.get(i);
            List<Point> points = detector.getLinePoints(line, 20);

            System.out.println("Right line " + i + " generated " + points.size() + " points");
        }
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
        assertFalse("Empty result should not have both camera detections", result.hasBothCameraDetections());
    }

    @Test
    public void testLineSegmentMethods() {
        // Test LineSegment utility methods
        Point start = new Point(100, 200);
        Point end = new Point(300, 400);
        RedLineDetector.LineSegment line = new RedLineDetector.LineSegment(start, end, "TEST");

        // Test center calculation
        Point center = line.getCenter();
        assertEquals("Center X should be average", 200.0, center.x, 0.1);
        assertEquals("Center Y should be average", 300.0, center.y, 0.1);

        // Test length calculation
        double expectedLength = Math.sqrt((300-100)*(300-100) + (400-200)*(400-200));
        assertEquals("Length should be calculated correctly", expectedLength, line.getLength(), 0.1);

        // Test angle calculation
        double expectedAngle = Math.atan2(400-200, 300-100);
        assertEquals("Angle should be calculated correctly", expectedAngle, line.getAngle(), 0.01);

        // Test toString method
        String lineStr = line.toString();
        assertNotNull("toString should not return null", lineStr);
        assertTrue("toString should contain camera ID", lineStr.contains("TEST"));
        assertTrue("toString should contain coordinates", lineStr.contains("100"));

        System.out.println("LineSegment toString: " + lineStr);
    }

    @Test
    public void testSyntheticImages() {
        // Test with synthetic images
        System.out.println("\n=== Testing with Synthetic Images ===");

        // Create synthetic test images
        Mat leftSynthetic = TestImageCreator.createSyntheticTestImage(640, 480, true);
        Mat rightSynthetic = TestImageCreator.createSyntheticTestImage(640, 480, true);

        assertNotNull("Left synthetic image should not be null", leftSynthetic);
        assertNotNull("Right synthetic image should not be null", rightSynthetic);
        assertFalse("Left synthetic image should not be empty", leftSynthetic.empty());
        assertFalse("Right synthetic image should not be empty", rightSynthetic.empty());

        // Test detection on synthetic images
        RedLineDetector.DetectionResult result = detector.detectRedLines(leftSynthetic, rightSynthetic);

        System.out.println("Synthetic image detection results:");
        System.out.println("  Left lines: " + result.leftLines.size());
        System.out.println("  Right lines: " + result.rightLines.size());

        // Save synthetic images for inspection
        saveVisualizationImage(leftSynthetic, "synthetic_left.jpg");
        saveVisualizationImage(rightSynthetic, "synthetic_right.jpg");

        // Create visualizations with detections
        Mat leftViz = leftSynthetic.clone();
        Mat rightViz = rightSynthetic.clone();

        detector.drawDetectedLines(leftViz, result.leftLines, new Scalar(0, 255, 0));
        detector.drawDetectedLines(rightViz, result.rightLines, new Scalar(0, 255, 0));

        saveVisualizationImage(leftViz, "synthetic_left_detection.jpg");
        saveVisualizationImage(rightViz, "synthetic_right_detection.jpg");

        // Clean up
        leftSynthetic.release();
        rightSynthetic.release();
        leftViz.release();
        rightViz.release();
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

            System.out.println("Running Red Line Detection Test Suite...");

            // Run all tests
            test.testImageLoadingOnly();
            test.testDetectorInitialization();
            test.testLineSegmentMethods();
            test.testColorMaskGeneration();
            test.testDebugMode();
            test.testLinePointsGeneration();
            test.testSyntheticImages();
            test.testRedLineDetection();

            test.tearDown();
            System.out.println("All tests completed successfully!");

        } catch (Exception e) {
            System.err.println("Test failed: " + e.getMessage());
            e.printStackTrace();
        }
    }
}

// Enhanced utility class for creating test images
class TestImageCreator {

    /**
     * Create a synthetic test image with red lines for testing
     */
    public static Mat createSyntheticTestImage(int width, int height, boolean withRedLines) {
        Mat image = new Mat(height, width, CvType.CV_8UC3, new Scalar(200, 200, 200));

        if (withRedLines) {
            // Add some red horizontal lines (similar to floor lines)
            Imgproc.line(image, new Point(50, height * 0.7), new Point(width - 50, height * 0.7),
                    new Scalar(0, 0, 255), 8);
            Imgproc.line(image, new Point(100, height * 0.8), new Point(width - 100, height * 0.8),
                    new Scalar(0, 0, 255), 6);

            // Add a diagonal red line
            Imgproc.line(image, new Point(width * 0.3, height * 0.6), new Point(width * 0.7, height * 0.9),
                    new Scalar(0, 0, 255), 5);

            // Add some noise to make it more realistic
            Mat noise = new Mat(height, width, CvType.CV_8UC3);
            Core.randn(noise, 0, 20);
            Core.add(image, noise, image);
            noise.release();
        }

        return image;
    }

    /**
     * Create a more complex synthetic image with multiple red features
     */
    public static Mat createComplexSyntheticImage(int width, int height) {
        Mat image = new Mat(height, width, CvType.CV_8UC3, new Scalar(180, 180, 180));

        // Add a main horizontal red line (like a floor line)
        Imgproc.line(image, new Point(0, height * 0.75), new Point(width, height * 0.75),
                new Scalar(0, 0, 255), 10);

        // Add some red rectangles (like objects)
        Imgproc.rectangle(image, new Point(width * 0.2, height * 0.3), new Point(width * 0.4, height * 0.5),
                new Scalar(0, 0, 255), -1);

        // Add a curved red line using connected line segments
        for (int i = 0; i < 20; i++) {
            double t = (double) i / 19.0;
            double x1 = width * 0.1 + t * width * 0.8;
            double y1 = height * 0.6 + 0.1 * height * Math.sin(t * Math.PI * 2);
            double x2 = width * 0.1 + (t + 0.05) * width * 0.8;
            double y2 = height * 0.6 + 0.1 * height * Math.sin((t + 0.05) * Math.PI * 2);

            Imgproc.line(image, new Point(x1, y1), new Point(x2, y2), new Scalar(0, 0, 255), 4);
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
            Mat complexImage = createComplexSyntheticImage(640, 480);

            // Save images
            Imgcodecs.imwrite("test/resources/RedlineDetector/test_left_1.jpg", leftImage);
            Imgcodecs.imwrite("test/resources/RedlineDetector/test_right_1.jpg", rightImage);
            Imgcodecs.imwrite("test/resources/RedlineDetector/test_complex.jpg", complexImage);

            System.out.println("Created synthetic test images");

            leftImage.release();
            rightImage.release();
            complexImage.release();

        } catch (Exception e) {
            System.err.println("Error creating test images: " + e.getMessage());
        }
    }
}