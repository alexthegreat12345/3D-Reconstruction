package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class RedLineDetector {

    // HSV color range for red detection
    private static final Scalar LOWER_RED_1 = new Scalar(0, 50, 50);
    private static final Scalar UPPER_RED_1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED_2 = new Scalar(160, 50, 50);
    private static final Scalar UPPER_RED_2 = new Scalar(180, 255, 255);

    // Line detection parameters
    private static final double MIN_LINE_LENGTH = 30;
    private static final double MAX_LINE_GAP = 10;
    private static final double RHO = 1;
    private static final double THETA = Math.PI / 180;
    private static final int THRESHOLD = 50;

    // Morphological operations kernels
    private Mat morphKernel;
    private Mat lineKernel;

    public RedLineDetector() {
        // Initialize morphological kernels
        morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        lineKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 3));
    }

    /**
     * Detects red lines in both camera frames
     * @param leftFrame Left camera frame
     * @param rightFrame Right camera frame
     * @return DetectionResult containing detected lines for both cameras
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
     * Detects red lines in a single frame
     * @param frame Input frame
     * @param cameraId Camera identifier for debugging
     * @return List of detected lines
     */
    private List<LineSegment> detectLinesInFrame(Mat frame, String cameraId) {
        List<LineSegment> detectedLines = new ArrayList<>();

        // Convert BGR to HSV for better color detection
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Create red mask (handling wrap-around in HSV)
        Mat redMask = createRedMask(hsvFrame);

        // Apply morphological operations to clean up the mask
        Mat cleanedMask = applyMorphology(redMask);

        // Detect lines using HoughLinesP
        Mat lines = new Mat();
        Imgproc.HoughLinesP(cleanedMask, lines, RHO, THETA, THRESHOLD,
                MIN_LINE_LENGTH, MAX_LINE_GAP);

        // Convert detected lines to LineSegment objects
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            if (line != null && line.length >= 4) {
                Point start = new Point(line[0], line[1]);
                Point end = new Point(line[2], line[3]);

                // Filter lines based on angle (horizontal lines on floor)
                if (isHorizontalLine(start, end)) {
                    LineSegment segment = new LineSegment(start, end, cameraId);
                    detectedLines.add(segment);
                }
            }
        }

        // Clean up temporary matrices
        hsvFrame.release();
        redMask.release();
        cleanedMask.release();
        lines.release();

        return detectedLines;
    }

    /**
     * Creates a binary mask for red color detection
     * @param hsvFrame HSV color space frame
     * @return Binary mask with red regions
     */
    private Mat createRedMask(Mat hsvFrame) {
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat combinedMask = new Mat();

        // Red color wraps around in HSV (0-10 and 160-180)
        Core.inRange(hsvFrame, LOWER_RED_1, UPPER_RED_1, mask1);
        Core.inRange(hsvFrame, LOWER_RED_2, UPPER_RED_2, mask2);

        // Combine both red ranges
        Core.bitwise_or(mask1, mask2, combinedMask);

        mask1.release();
        mask2.release();

        return combinedMask;
    }

    /**
     * Applies morphological operations to clean up the mask
     * @param mask Input binary mask
     * @return Cleaned binary mask
     */
    private Mat applyMorphology(Mat mask) {
        Mat temp = new Mat();
        Mat result = new Mat();

        // Remove noise with opening operation
        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_OPEN, morphKernel);

        // Enhance line structure with closing operation
        Imgproc.morphologyEx(temp, result, Imgproc.MORPH_CLOSE, lineKernel);

        temp.release();

        return result;
    }

    /**
     * Checks if a line is approximately horizontal (floor line)
     * @param start Start point of line
     * @param end End point of line
     * @return True if line is horizontal within tolerance
     */
    private boolean isHorizontalLine(Point start, Point end) {
        double angle = Math.atan2(end.y - start.y, end.x - start.x);
        double angleDegrees = Math.toDegrees(Math.abs(angle));

        // Allow lines within ±20 degrees of horizontal
        return angleDegrees <= 20 || angleDegrees >= 160;
    }

    /**
     * Draws detected lines on the frame for visualization
     * @param frame Input frame
     * @param lines List of detected lines
     * @param color Color for drawing lines
     */
    public void drawDetectedLines(Mat frame, List<LineSegment> lines, Scalar color) {
        for (LineSegment line : lines) {
            Imgproc.line(frame, line.start, line.end, color, 3);

            // Draw center point
            Point center = line.getCenter();
            Imgproc.circle(frame, center, 5, new Scalar(0, 255, 0), -1);
        }
    }

    /**
     * Filters lines based on position (floor region)
     * @param lines Input lines
     * @param frameHeight Height of the frame
     * @return Filtered lines likely to be on the floor
     */
    public List<LineSegment> filterFloorLines(List<LineSegment> lines, int frameHeight) {
        List<LineSegment> floorLines = new ArrayList<>();

        // Consider bottom 40% of frame as floor region
        double floorThreshold = frameHeight * 0.6;

        for (LineSegment line : lines) {
            Point center = line.getCenter();
            if (center.y >= floorThreshold) {
                floorLines.add(line);
            }
        }

        return floorLines;
    }

    /**
     * Class to represent a line segment
     */
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

    /**
     * Class to hold detection results from both cameras
     */
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

    /**
     * Release resources
     */
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