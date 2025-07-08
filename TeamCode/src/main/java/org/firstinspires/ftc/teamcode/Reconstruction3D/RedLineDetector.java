package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class RedLineDetector {

    // More flexible HSV color ranges for red detection
    private static final Scalar LOWER_RED_1 = new Scalar(0, 30, 30);    // Lowered saturation/value thresholds
    private static final Scalar UPPER_RED_1 = new Scalar(15, 255, 255); // Expanded hue range
    private static final Scalar LOWER_RED_2 = new Scalar(160, 30, 30);  // Lowered saturation/value thresholds
    private static final Scalar UPPER_RED_2 = new Scalar(180, 255, 255);

    // Adjusted line detection parameters
    private static final double MIN_LINE_LENGTH = 20;  // Reduced minimum length
    private static final double MAX_LINE_GAP = 15;     // Increased gap tolerance
    private static final double RHO = 1;
    private static final double THETA = Math.PI / 180;
    private static final int THRESHOLD = 30;           // Reduced threshold

    // Smaller morphological operations kernels
    private Mat morphKernel;
    private Mat lineKernel;

    private boolean debugMode = false;

    public RedLineDetector() {
        // Smaller kernels for better line preservation
        morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        lineKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 2));
    }

    public void setDebugMode(boolean debug) {
        this.debugMode = debug;
    }

    /**
     * Detects red lines in both camera frames
     */
    public DetectionResult detectRedLines(Mat leftFrame, Mat rightFrame) {
        DetectionResult result = new DetectionResult();

        // Process left camera frame
        result.leftLines = detectLinesInFrame(leftFrame, "LEFT");

        // Process right camera frame
        result.rightLines = detectLinesInFrame(rightFrame, "RIGHT");

        return result;
    }

    /**
     * Enhanced line detection with debugging support
     */
    private List<LineSegment> detectLinesInFrame(Mat frame, String cameraId) {
        List<LineSegment> detectedLines = new ArrayList<>();

        if (debugMode) {
            System.out.println("Processing frame for camera: " + cameraId);
            System.out.println("Frame size: " + frame.size());
        }

        // Convert BGR to HSV for better color detection
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Create red mask (handling wrap-around in HSV)
        Mat redMask = createRedMask(hsvFrame);

        if (debugMode) {
            // Count non-zero pixels in mask
            int nonZeroPixels = Core.countNonZero(redMask);
            System.out.println("Non-zero pixels in red mask: " + nonZeroPixels);
        }

        // Apply morphological operations to clean up the mask
        Mat cleanedMask = applyMorphology(redMask);

        if (debugMode) {
            int cleanedNonZero = Core.countNonZero(cleanedMask);
            System.out.println("Non-zero pixels after morphology: " + cleanedNonZero);
        }

        // Detect lines using HoughLinesP
        Mat lines = new Mat();
        Imgproc.HoughLinesP(cleanedMask, lines, RHO, THETA, THRESHOLD,
                MIN_LINE_LENGTH, MAX_LINE_GAP);

        if (debugMode) {
            System.out.println("Raw lines detected: " + lines.rows());
        }

        // Convert detected lines to LineSegment objects
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            if (line != null && line.length >= 4) {
                Point start = new Point(line[0], line[1]);
                Point end = new Point(line[2], line[3]);

                LineSegment segment = new LineSegment(start, end, cameraId);

                if (debugMode) {
                    System.out.println("Line " + i + ": " +
                            "Start(" + start.x + "," + start.y + ") " +
                            "End(" + end.x + "," + end.y + ") " +
                            "Length: " + segment.getLength() +
                            " Angle: " + Math.toDegrees(segment.getAngle()));
                }

                // Less restrictive horizontal filter
                if (isHorizontalLine(start, end)) {
                    detectedLines.add(segment);
                } else if (debugMode) {
                    System.out.println("Line rejected due to angle filter");
                }
            }
        }

        if (debugMode) {
            System.out.println("Final detected lines: " + detectedLines.size());
        }

        // Clean up temporary matrices
        hsvFrame.release();
        redMask.release();
        cleanedMask.release();
        lines.release();

        return detectedLines;
    }

    /**
     * Creates a binary mask for red color detection with multiple ranges
     */
    private Mat createRedMask(Mat hsvFrame) {
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask3 = new Mat();  // Additional range for orange-red
        Mat combinedMask = new Mat();

        // Primary red ranges
        Core.inRange(hsvFrame, LOWER_RED_1, UPPER_RED_1, mask1);
        Core.inRange(hsvFrame, LOWER_RED_2, UPPER_RED_2, mask2);

        // Additional range for orange-red colors
        Core.inRange(hsvFrame, new Scalar(15, 30, 30), new Scalar(25, 255, 255), mask3);

        // Combine all red ranges
        Core.bitwise_or(mask1, mask2, combinedMask);
        Core.bitwise_or(combinedMask, mask3, combinedMask);

        mask1.release();
        mask2.release();
        mask3.release();

        return combinedMask;
    }

    /**
     * Gentler morphological operations
     */
    private Mat applyMorphology(Mat mask) {
        Mat temp = new Mat();
        Mat result = new Mat();

        // Gentle noise removal
        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_OPEN, morphKernel);

        // Mild line enhancement
        Imgproc.morphologyEx(temp, result, Imgproc.MORPH_CLOSE, lineKernel);

        temp.release();
        return result;
    }

    /**
     * More flexible horizontal line detection
     */
    private boolean isHorizontalLine(Point start, Point end) {
        double angle = Math.atan2(end.y - start.y, end.x - start.x);
        double angleDegrees = Math.toDegrees(Math.abs(angle));

        // More permissive angle tolerance (±30 degrees)
        return angleDegrees <= 30 || angleDegrees >= 150;
    }

    /**
     * Alternative detection method with edge detection
     */
    public List<LineSegment> detectLinesWithEdges(Mat frame, String cameraId) {
        List<LineSegment> detectedLines = new ArrayList<>();

        // Convert to HSV and create red mask
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        Mat redMask = createRedMask(hsvFrame);

        // Apply Gaussian blur to reduce noise
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(redMask, blurred, new Size(5, 5), 0);

        // Apply Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(blurred, edges, 50, 150);

        // Detect lines using HoughLinesP with different parameters
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI/180, 25, 15, 20);

        // Process detected lines
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            if (line != null && line.length >= 4) {
                Point start = new Point(line[0], line[1]);
                Point end = new Point(line[2], line[3]);

                LineSegment segment = new LineSegment(start, end, cameraId);
                if (isHorizontalLine(start, end)) {
                    detectedLines.add(segment);
                }
            }
        }

        // Clean up
        hsvFrame.release();
        redMask.release();
        blurred.release();
        edges.release();
        lines.release();

        return detectedLines;
    }

    /**
     * Test method to check color detection
     */
    public Mat getColorMask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        Mat mask = createRedMask(hsvFrame);
        hsvFrame.release();
        return mask;
    }

    /**
     * Draws detected lines on the frame for visualization
     */
    public void drawDetectedLines(Mat frame, List<LineSegment> lines, Scalar color) {
        for (LineSegment line : lines) {
            Imgproc.line(frame, line.start, line.end, color, 3);

            // Draw center point
            Point center = line.getCenter();
            Imgproc.circle(frame, center, 5, new Scalar(0, 255, 0), -1);

            // Draw endpoints
            Imgproc.circle(frame, line.start, 3, new Scalar(255, 0, 0), -1);
            Imgproc.circle(frame, line.end, 3, new Scalar(0, 0, 255), -1);
        }
    }

    /**
     * Filters lines based on position (floor region)
     */
    public List<LineSegment> filterFloorLines(List<LineSegment> lines, int frameHeight) {
        List<LineSegment> floorLines = new ArrayList<>();

        // Consider bottom 50% of frame as floor region (less restrictive)
        double floorThreshold = frameHeight * 0.5;

        for (LineSegment line : lines) {
            Point center = line.getCenter();
            if (center.y >= floorThreshold) {
                floorLines.add(line);
            }
        }

        return floorLines;
    }

    // Rest of the classes remain the same...
    public static class LineSegment {
        public Point start;
        public Point end;
        public String cameraId;

        public LineSegment(Point start, Point end, String cameraId) {
            this.start = start;
            this.end = end;
            this.cameraId = cameraId;
        }

        public Point getCenter() {
            return new Point((start.x + end.x) / 2, (start.y + end.y) / 2);
        }

        public double getLength() {
            return Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));
        }

        public double getAngle() {
            return Math.atan2(end.y - start.y, end.x - start.x);
        }
    }

    public static class DetectionResult {
        public List<LineSegment> leftLines;
        public List<LineSegment> rightLines;

        public DetectionResult() {
            leftLines = new ArrayList<>();
            rightLines = new ArrayList<>();
        }

        public boolean hasDetections() {
            return !leftLines.isEmpty() || !rightLines.isEmpty();
        }

        public int getTotalDetections() {
            return leftLines.size() + rightLines.size();
        }
    }

    public void release() {
        if (morphKernel != null) {
            morphKernel.release();
        }
        if (lineKernel != null) {
            lineKernel.release();
        }
    }
}

// Example usage in your FTC OpMode:
/*
public class StereoVisionOpMode extends LinearOpMode {
    private RedLineDetector detector;

    @Override
    public void runOpMode() {
        // Initialize detector
        detector = new RedLineDetector();

        waitForStart();

        while (opModeIsActive()) {
            // Get frames from both cameras
            Mat leftFrame = getLeftCameraFrame();
            Mat rightFrame = getRightCameraFrame();

            // Detect red lines
            RedLineDetector.DetectionResult result = detector.detectRedLines(leftFrame, rightFrame);

            // Filter for floor lines
            List<RedLineDetector.LineSegment> leftFloorLines =
                detector.filterFloorLines(result.leftLines, leftFrame.rows());
            List<RedLineDetector.LineSegment> rightFloorLines =
                detector.filterFloorLines(result.rightLines, rightFrame.rows());

            // Draw results for visualization
            detector.drawDetectedLines(leftFrame, leftFloorLines, new Scalar(0, 0, 255));
            detector.drawDetectedLines(rightFrame, rightFloorLines, new Scalar(0, 0, 255));

            telemetry.addData("Left Lines", leftFloorLines.size());
            telemetry.addData("Right Lines", rightFloorLines.size());
            telemetry.update();

            // TODO: Next step - triangulate 3D coordinates from corresponding lines
        }

        detector.release();
    }
}
*/