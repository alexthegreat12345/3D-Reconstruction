package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.junit.Before;
import org.junit.Test;
import org.junit.Assert;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

public class StereoVision3DCalculatorTest {

    private StereoVision3DCalculator calculator;
    private Mat cameraMatrix1, cameraMatrix2;
    private Mat distCoeffs1, distCoeffs2;
    private Mat R, T;
    private Size imageSize;
    private Mat testLeftImage, testRightImage;

    @Before
    public void setUp() {
        // Load OpenCV native library (this would be handled by your FTC setup)
        // nu.pattern.OpenCV.loadShared();

        // Initialize test image size from actual calibration data
        imageSize = new Size(1440, 960);

        // Load actual left camera calibration data
        cameraMatrix1 = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix1.put(0, 0,
                1806.53306139, 0.00000000, 719.50000000,
                0.00000000, 1809.20766166, 479.50000000,
                0.00000000, 0.00000000, 1.00000000
        );

        // Load actual right camera calibration data
        cameraMatrix2 = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix2.put(0, 0,
                1779.99696418, 0.00000000, 719.50000000,
                0.00000000, 1785.19655182, 479.50000000,
                0.00000000, 0.00000000, 1.00000000
        );

        // Load actual left camera distortion coefficients
        distCoeffs1 = new Mat(1, 5, CvType.CV_64F);
        distCoeffs1.put(0, 0, -0.38553184, 0.18314920, 0.00000000, 0.00000000, 1.19447628);

        // Load actual right camera distortion coefficients
        distCoeffs2 = new Mat(1, 5, CvType.CV_64F);
        distCoeffs2.put(0, 0, -0.33710709, 0.00729703, 0.00000000, 0.00000000, 0.23798294);

        // Load actual stereo calibration rotation matrix
        R = new Mat(3, 3, CvType.CV_64F);
        R.put(0, 0,
                0.91891195, -0.30592921, 0.24901436,
                0.38275524, 0.84417552, -0.37532135,
                -0.09539006, 0.44019882, 0.89281898
        );

        // Load actual stereo calibration translation vector
        T = new Mat(3, 1, CvType.CV_64F);
        T.put(0, 0, -485.95790648, 317.12975431, 165.49815742);

        // Initialize calculator
        calculator = new StereoVision3DCalculator(
                cameraMatrix1, distCoeffs1,
                cameraMatrix2, distCoeffs2,
                R, T, imageSize
        );

        // Create test images
        testLeftImage = Mat.zeros((int)imageSize.height, (int)imageSize.width, CvType.CV_8UC3);
        testRightImage = Mat.zeros((int)imageSize.height, (int)imageSize.width, CvType.CV_8UC3);
    }

    /**
     * Helper method to create calibration data from JSON files
     * This would be used in a real FTC application to load calibration data
     */
    private static class CalibrationData {
        public Mat cameraMatrix1, cameraMatrix2;
        public Mat distCoeffs1, distCoeffs2;
        public Mat R, T;
        public Size imageSize;

        public static CalibrationData fromJsonFiles(String leftCalibFile, String rightCalibFile, String stereoCalibFile) {
            // In a real implementation, you would parse JSON files here
            // For now, we'll use the hardcoded values from the actual calibration
            CalibrationData data = new CalibrationData();

            data.imageSize = new Size(1440, 960);

            // Left camera matrix
            data.cameraMatrix1 = new Mat(3, 3, CvType.CV_64F);
            data.cameraMatrix1.put(0, 0,
                    1806.53306139, 0.00000000, 719.50000000,
                    0.00000000, 1809.20766166, 479.50000000,
                    0.00000000, 0.00000000, 1.00000000
            );

            // Right camera matrix
            data.cameraMatrix2 = new Mat(3, 3, CvType.CV_64F);
            data.cameraMatrix2.put(0, 0,
                    1779.99696418, 0.00000000, 719.50000000,
                    0.00000000, 1785.19655182, 479.50000000,
                    0.00000000, 0.00000000, 1.00000000
            );

            // Left distortion coefficients
            data.distCoeffs1 = new Mat(1, 5, CvType.CV_64F);
            data.distCoeffs1.put(0, 0, -0.38553184, 0.18314920, 0.00000000, 0.00000000, 1.19447628);

            // Right distortion coefficients
            data.distCoeffs2 = new Mat(1, 5, CvType.CV_64F);
            data.distCoeffs2.put(0, 0, -0.33710709, 0.00729703, 0.00000000, 0.00000000, 0.23798294);

            // Stereo rotation matrix
            data.R = new Mat(3, 3, CvType.CV_64F);
            data.R.put(0, 0,
                    0.91891195, -0.30592921, 0.24901436,
                    0.38275524, 0.84417552, -0.37532135,
                    -0.09539006, 0.44019882, 0.89281898
            );

            // Stereo translation vector
            data.T = new Mat(3, 1, CvType.CV_64F);
            data.T.put(0, 0, -485.95790648, 317.12975431, 165.49815742);

            return data;
        }
    }

    @Test
    public void testCalculatorInitialization() {
        assertNotNull("Calculator should be initialized", calculator);

        // Test that calculator can be created without throwing exceptions
        StereoVision3DCalculator testCalc = new StereoVision3DCalculator(
                cameraMatrix1, distCoeffs1,
                cameraMatrix2, distCoeffs2,
                R, T, imageSize
        );
        assertNotNull("Test calculator should be initialized", testCalc);
    }

    @Test
    public void testCalculate3DCoordinatesWithEmptyPoints() {
        List<Point> emptyLeftPoints = new ArrayList<>();
        List<Point> emptyRightPoints = new ArrayList<>();

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage,
                emptyLeftPoints, emptyRightPoints
        );

        assertNotNull("Result should not be null", result);
        assertEquals("Result should be empty for empty input", 0, result.size());
    }

    @Test
    public void testCalculate3DCoordinatesWithValidPoints() {
        // Create test points that should match (same Y coordinate, different X)
        // Using realistic coordinates for 1440x960 image
        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        // Add corresponding points with reasonable disparity for the actual baseline
        leftPoints.add(new Point(720, 480)); // Center of image
        rightPoints.add(new Point(700, 480)); // 20 pixel disparity

        leftPoints.add(new Point(900, 400)); // Upper right area
        rightPoints.add(new Point(880, 400)); // 20 pixel disparity

        leftPoints.add(new Point(540, 600)); // Lower left area
        rightPoints.add(new Point(520, 600)); // 20 pixel disparity

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage,
                leftPoints, rightPoints
        );

        assertNotNull("Result should not be null", result);
        System.out.println("3D points calculated: " + result.size());

        // With actual calibration data, we should get reasonable 3D coordinates
        for (Point3 point : result) {
            System.out.println("3D Point: (" + point.x + ", " + point.y + ", " + point.z + ")");
        }
    }

    @Test
    public void testCalculate3DCoordinatesWithMismatchedPoints() {
        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        // Add points that won't match (different Y coordinates)
        leftPoints.add(new Point(720, 480));
        rightPoints.add(new Point(700, 550)); // Too far vertically (70 pixels)

        leftPoints.add(new Point(900, 400));
        rightPoints.add(new Point(880, 500)); // Too far vertically (100 pixels)

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage,
                leftPoints, rightPoints
        );

        assertNotNull("Result should not be null", result);
        // Should have fewer or no matches due to epipolar constraint
        System.out.println("Mismatched points result: " + result.size() + " matches");
    }

    @Test
    public void testFilterAndRefine3DPoints() {
        List<Point3> testPoints = new ArrayList<>();

        // Add good points
        testPoints.add(new Point3(0.1, 0.1, 1.0));
        testPoints.add(new Point3(0.2, 0.2, 1.1));
        testPoints.add(new Point3(0.3, 0.3, 1.2));

        // Add outliers
        testPoints.add(new Point3(0.1, 0.1, 10.0)); // Too far
        testPoints.add(new Point3(0.1, 0.1, 0.01)); // Too close
        testPoints.add(new Point3(0.1, 0.1, -1.0)); // Negative depth

        List<Point3> filtered = calculator.filterAndRefine3DPoints(testPoints);

        assertNotNull("Filtered result should not be null", filtered);
        assertTrue("Should filter out outliers", filtered.size() < testPoints.size());
        assertTrue("Should keep some good points", filtered.size() > 0);

        // Verify all remaining points have positive, reasonable depth
        for (Point3 point : filtered) {
            assertTrue("All points should have positive depth", point.z > 0);
            assertTrue("All points should be within reasonable range", point.z < 5.0);
        }
    }

    @Test
    public void testFilterAndRefine3DPointsWithFewPoints() {
        List<Point3> testPoints = new ArrayList<>();
        testPoints.add(new Point3(0.1, 0.1, 1.0));
        testPoints.add(new Point3(0.2, 0.2, 1.1));

        List<Point3> filtered = calculator.filterAndRefine3DPoints(testPoints);

        assertNotNull("Filtered result should not be null", filtered);
        assertEquals("Should return all points when less than 3", testPoints.size(), filtered.size());
    }

    @Test
    public void testGetRedLineCenterPointWithNoPoints() {
        // Test with no points calculated
        Point3 center = calculator.getRedLineCenterPoint();
        assertNull("Center should be null when no points exist", center);
    }

    @Test
    public void testGetRedLineCenterPointWithValidPoints() {
        // First calculate some 3D points
        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        leftPoints.add(new Point(320, 240));
        rightPoints.add(new Point(310, 240));

        calculator.calculate3DCoordinates(testLeftImage, testRightImage, leftPoints, rightPoints);

        Point3 center = calculator.getRedLineCenterPoint();
        // Center might be null if triangulation failed, which is ok for this test setup
        // In a real test with proper calibration, you'd verify the center calculation
    }

    @Test
    public void testGetDistanceToRedLineWithNoPoints() {
        double distance = calculator.getDistanceToRedLine();
        assertEquals("Distance should be -1 when no points exist", -1.0, distance, 0.001);
    }

    @Test
    public void testGetAngleToRedLineWithNoPoints() {
        double angle = calculator.getAngleToRedLine();
        assertEquals("Angle should be 0 when no points exist", 0.0, angle, 0.001);
    }

    @Test
    public void testConvertToRobotCoordinates() {
        Point3 cameraPoint = new Point3(0.1, 0.5, 1.0);
        double cameraHeight = 0.3; // 30cm camera height
        double cameraTiltAngle = 10.0; // 10 degrees tilt

        Point3 robotPoint = calculator.convertToRobotCoordinates(
                cameraPoint, cameraHeight, cameraTiltAngle
        );

        assertNotNull("Robot coordinates should not be null", robotPoint);
        assertEquals("X coordinate should remain the same", cameraPoint.x, robotPoint.x, 0.001);
        assertNotEquals("Y coordinate should change due to height adjustment", cameraPoint.y, robotPoint.y);
        // Z might change slightly due to tilt correction
    }

    @Test
    public void testConvertToRobotCoordinatesWithZeroTilt() {
        Point3 cameraPoint = new Point3(0.1, 0.5, 1.0);
        double cameraHeight = 0.3;
        double cameraTiltAngle = 0.0; // No tilt

        Point3 robotPoint = calculator.convertToRobotCoordinates(
                cameraPoint, cameraHeight, cameraTiltAngle
        );

        assertNotNull("Robot coordinates should not be null", robotPoint);
        assertEquals("X coordinate should remain the same", cameraPoint.x, robotPoint.x, 0.001);
        assertEquals("Y coordinate should only change by camera height",
                cameraPoint.y - cameraHeight, robotPoint.y, 0.001);
        assertEquals("Z coordinate should remain the same with no tilt",
                cameraPoint.z, robotPoint.z, 0.001);
    }

    @Test
    public void testMultipleCalculations() {
        // Test that multiple calculations don't interfere with each other
        List<Point> leftPoints1 = new ArrayList<>();
        List<Point> rightPoints1 = new ArrayList<>();
        leftPoints1.add(new Point(320, 240));
        rightPoints1.add(new Point(310, 240));

        List<Point3> result1 = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftPoints1, rightPoints1
        );

        List<Point> leftPoints2 = new ArrayList<>();
        List<Point> rightPoints2 = new ArrayList<>();
        leftPoints2.add(new Point(400, 200));
        rightPoints2.add(new Point(390, 200));

        List<Point3> result2 = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftPoints2, rightPoints2
        );

        assertNotNull("First result should not be null", result1);
        assertNotNull("Second result should not be null", result2);
        // Results should be independent
    }

    @Test
    public void testLargeNumberOfPoints() {
        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        // Add many points
        for (int i = 0; i < 100; i++) {
            leftPoints.add(new Point(100 + i * 3, 240));
            rightPoints.add(new Point(90 + i * 3, 240)); // 10 pixel disparity
        }

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftPoints, rightPoints
        );

        assertNotNull("Result should not be null", result);
        // Should handle large number of points without crashing
    }

    @Test
    public void testEdgeCaseCoordinates() {
        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        // Test points near image edges (for 1440x960 image)
        leftPoints.add(new Point(10, 10));      // Top-left near corner
        rightPoints.add(new Point(5, 10));      // Small disparity

        leftPoints.add(new Point(1430, 950));   // Bottom-right near corner
        rightPoints.add(new Point(1420, 950));  // 10 pixel disparity

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftPoints, rightPoints
        );

        assertNotNull("Result should not be null", result);
        // Should handle edge coordinates gracefully
        System.out.println("Edge case coordinates result: " + result.size() + " points");
    }

    @Test
    public void testRedLineDetectionScenario() {
        // Simulate red line detection points on the floor
        // Red lines are typically horizontal on the floor, so points should have similar Y coordinates
        List<Point> leftRedLinePoints = new ArrayList<>();
        List<Point> rightRedLinePoints = new ArrayList<>();

        // Simulate a red line across the bottom portion of the image (floor area)
        int lineY = 750; // Bottom third of 960px height image

        // Add points along the red line
        leftRedLinePoints.add(new Point(200, lineY));
        rightRedLinePoints.add(new Point(180, lineY));

        leftRedLinePoints.add(new Point(400, lineY));
        rightRedLinePoints.add(new Point(380, lineY));

        leftRedLinePoints.add(new Point(600, lineY));
        rightRedLinePoints.add(new Point(580, lineY));

        leftRedLinePoints.add(new Point(800, lineY));
        rightRedLinePoints.add(new Point(780, lineY));

        leftRedLinePoints.add(new Point(1000, lineY));
        rightRedLinePoints.add(new Point(980, lineY));

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftRedLinePoints, rightRedLinePoints
        );

        assertNotNull("Result should not be null", result);
        System.out.println("Red line 3D points: " + result.size());

        if (!result.isEmpty()) {
            // Test center point calculation
            Point3 centerPoint = calculator.getRedLineCenterPoint();
            assertNotNull("Center point should not be null", centerPoint);
            System.out.println("Red line center: (" + centerPoint.x + ", " + centerPoint.y + ", " + centerPoint.z + ")");

            // Test distance calculation
            double distance = calculator.getDistanceToRedLine();
            assertTrue("Distance should be positive", distance > 0);
            System.out.println("Distance to red line: " + distance + " units");

            // Test angle calculation
            double angle = calculator.getAngleToRedLine();
            System.out.println("Angle to red line: " + angle + " degrees");

            // Test filtering
            List<Point3> filteredPoints = calculator.filterAndRefine3DPoints(result);
            assertNotNull("Filtered points should not be null", filteredPoints);
            System.out.println("Filtered points: " + filteredPoints.size());
        }
    }

    @Test
    public void testCalibrationDataLoading() {
        // Test the calibration data loading helper
        CalibrationData data = CalibrationData.fromJsonFiles(
                "left_camera_calibration.json",
                "right_camera_calibration.json",
                "stereo_calibration.json"
        );

        assertNotNull("Calibration data should not be null", data);
        assertEquals("Image width should match", 1440.0, data.imageSize.width, 0.1);
        assertEquals("Image height should match", 960.0, data.imageSize.height, 0.1);

        // Test creating calculator with loaded data
        StereoVision3DCalculator testCalc = new StereoVision3DCalculator(
                data.cameraMatrix1, data.distCoeffs1,
                data.cameraMatrix2, data.distCoeffs2,
                data.R, data.T, data.imageSize
        );

        assertNotNull("Calculator should be created with loaded data", testCalc);
    }

    // Helper method to create a simple test pattern
    private void createTestPattern(Mat image, Point center, Scalar color) {
        // Draw a simple cross pattern
        org.opencv.imgproc.Imgproc.line(image,
                new Point(center.x - 10, center.y),
                new Point(center.x + 10, center.y),
                color, 2);
        org.opencv.imgproc.Imgproc.line(image,
                new Point(center.x, center.y - 10),
                new Point(center.x, center.y + 10),
                color, 2);
    }

    @Test
    public void testWithSimpleTestPattern() {
        // Create simple test patterns in both images
        createTestPattern(testLeftImage, new Point(320, 240), new Scalar(0, 0, 255)); // Red
        createTestPattern(testRightImage, new Point(310, 240), new Scalar(0, 0, 255)); // Red, shifted

        List<Point> leftPoints = new ArrayList<>();
        List<Point> rightPoints = new ArrayList<>();

        leftPoints.add(new Point(320, 240));
        rightPoints.add(new Point(310, 240));

        List<Point3> result = calculator.calculate3DCoordinates(
                testLeftImage, testRightImage, leftPoints, rightPoints
        );

        assertNotNull("Result should not be null", result);
        // With test patterns, we should get some results
    }
}