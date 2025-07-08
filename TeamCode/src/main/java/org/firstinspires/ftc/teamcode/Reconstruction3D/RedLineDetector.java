package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

public class RedLineDetector {

    // More restrictive HSV color ranges based on Python working code
    private static final Scalar LOWER_RED_1 = new Scalar(0, 50, 50);     // Higher saturation/value
    private static final Scalar UPPER_RED_1 = new Scalar(10, 255, 255);  // Narrower hue range
    private static final Scalar LOWER_RED_2 = new Scalar(170, 50, 50);   // Higher saturation/value
    private static final Scalar UPPER_RED_2 = new Scalar(180, 255, 255);

    // Much more restrictive line detection parameters
    private static final double MIN_LINE_LENGTH = 100;   // Significantly increased
    private static final double MAX_LINE_GAP = 10;       // Slightly increased
    private static final double RHO = 1;
    private static final double THETA = Math.PI / 180;
    private static final int THRESHOLD = 50;             // Lowered for better detection

    // Morphological operation kernels
    private Mat morphKernel;
    private Mat closeKernel;

    private boolean debugMode = false;

    public RedLineDetector() {
        // Kernels matching Python approach
        morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    }

    public void setDebugMode(boolean debug) {
        this.debugMode = debug;
    }

    /**
     * Main detection method - detects red lines in both camera frames
     */
    public DetectionResult detectRedLines(Mat leftFrame, Mat rightFrame) {
        DetectionResult result = new DetectionResult();

        // Process each frame and get the best line from each
        result.leftLines = detectBestLineInFrame(leftFrame, "LEFT");
        result.rightLines = detectBestLineInFrame(rightFrame, "RIGHT");

        return result;
    }

    /**
     * Detects the single best red line in a frame (similar to Python approach)
     */
    private List<LineSegment> detectBestLineInFrame(Mat frame, String cameraId) {
        List<LineSegment> result = new ArrayList<>();

        if (debugMode) {
            System.out.println("Processing frame for camera: " + cameraId);
            System.out.println("Frame size: " + frame.size());
        }

        // Convert BGR to HSV
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Create red mask using both red ranges
        Mat redMask = createRedMask(hsvFrame);

        if (debugMode) {
            int nonZeroPixels = Core.countNonZero(redMask);
            System.out.println("Non-zero pixels in red mask: " + nonZeroPixels);
        }

        // Apply morphological operations (matching Python approach)
        Mat cleanedMask = applyMorphologyPythonStyle(redMask);

        // Find contours instead of direct line detection
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(cleanedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (debugMode) {
            System.out.println("Found " + contours.size() + " contours");
        }

        // Filter contours by area and aspect ratio (like Python code)
        List<MatOfPoint> lineContours = filterLineContours(contours);

        if (debugMode) {
            System.out.println("Detected " + lineContours.size() + " potential line contours");
        }

        // Find the largest contour and fit a line through it
        if (!lineContours.isEmpty()) {
            MatOfPoint largestContour = findLargestContour(lineContours);
            LineSegment bestLine = fitLineToContour(largestContour, cameraId, frame.size());

            if (bestLine != null) {
                result.add(bestLine);
                if (debugMode) {
                    System.out.println("Best line found: " + bestLine.toString());
                }
            }
        }

        // Clean up
        hsvFrame.release();
        redMask.release();
        cleanedMask.release();
        hierarchy.release();
        for (MatOfPoint contour : contours) {
            contour.release();
        }

        return result;
    }

    /**
     * Creates red mask with both red ranges
     */
    private Mat createRedMask(Mat hsvFrame) {
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat combinedMask = new Mat();

        // Create masks for both red ranges
        Core.inRange(hsvFrame, LOWER_RED_1, UPPER_RED_1, mask1);
        Core.inRange(hsvFrame, LOWER_RED_2, UPPER_RED_2, mask2);

        // Combine masks
        Core.bitwise_or(mask1, mask2, combinedMask);

        mask1.release();
        mask2.release();

        return combinedMask;
    }

    /**
     * Apply morphological operations similar to Python code
     */
    private Mat applyMorphologyPythonStyle(Mat mask) {
        Mat temp = new Mat();
        Mat result = new Mat();

        // Close operation first (fill small gaps)
        Imgproc.morphologyEx(mask, temp, Imgproc.MORPH_CLOSE, closeKernel);

        // Then open operation (remove small noise)
        Imgproc.morphologyEx(temp, result, Imgproc.MORPH_OPEN, morphKernel);

        temp.release();
        return result;
    }

    /**
     * Filter contours by area and aspect ratio (matching Python logic)
     */
    private List<MatOfPoint> filterLineContours(List<MatOfPoint> contours) {
        List<MatOfPoint> lineContours = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > 100) {  // Minimum area threshold (same as Python)
                Rect boundingRect = Imgproc.boundingRect(contour);
                double aspectRatio = Math.max(boundingRect.width, boundingRect.height) /
                        (double) Math.min(boundingRect.width, boundingRect.height);

                if (debugMode) {
                    System.out.println("Contour area: " + area + ", Aspect ratio: " + String.format("%.2f", aspectRatio));
                }

                // Check if it's line-like (high aspect ratio, same as Python)
                if (aspectRatio > 2.5) {
                    lineContours.add(contour);
                }
            }
        }

        return lineContours;
    }

    /**
     * Find the largest contour (assuming it's the main red line)
     */
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        MatOfPoint largest = null;
        double maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largest = contour;
            }
        }

        return largest;
    }

    /**
     * Fit a line through the contour points (similar to Python cv2.fitLine)
     */
    private LineSegment fitLineToContour(MatOfPoint contour, String cameraId, Size frameSize) {
        Point[] contourPoints = contour.toArray();

        if (contourPoints.length < 10) {
            if (debugMode) {
                System.out.println("Not enough points to fit a line");
            }
            return null;
        }

        // Convert points to Mat for fitLine
        Mat pointsMat = new Mat(contourPoints.length, 1, CvType.CV_32FC2);
        for (int i = 0; i < contourPoints.length; i++) {
            pointsMat.put(i, 0, contourPoints[i].x, contourPoints[i].y);
        }

        // Fit line using OpenCV fitLine
        Mat lineMat = new Mat();
        Imgproc.fitLine(pointsMat, lineMat, Imgproc.DIST_L2, 0, 0.01, 0.01);

        // Extract line parameters [vx, vy, x0, y0]
        double[] lineParams = lineMat.get(0, 0);
        double vx = lineParams[0];
        double vy = lineParams[1];
        double x0 = lineParams[2];
        double y0 = lineParams[3];

        // Calculate line endpoints using parametric equation
        Point startPoint = null;
        Point endPoint = null;

        if (Math.abs(vx) > 1e-6) {  // Avoid division by zero
            // Calculate t for left and right edges
            double tLeft = -x0 / vx;
            double tRight = (frameSize.width - 1 - x0) / vx;

            int leftY = (int) Math.round(y0 + tLeft * vy);
            int rightY = (int) Math.round(y0 + tRight * vy);

            // Clamp y coordinates
            leftY = Math.max(0, Math.min((int) frameSize.height - 1, leftY));
            rightY = Math.max(0, Math.min((int) frameSize.height - 1, rightY));

            startPoint = new Point(0, leftY);
            endPoint = new Point(frameSize.width - 1, rightY);
        } else {
            // Handle vertical line case
            int x = (int) Math.round(x0);
            if (x >= 0 && x < frameSize.width) {
                startPoint = new Point(x, 0);
                endPoint = new Point(x, frameSize.height - 1);
            }
        }

        // Clean up
        pointsMat.release();
        lineMat.release();

        if (startPoint != null && endPoint != null) {
            return new LineSegment(startPoint, endPoint, cameraId);
        }

        return null;
    }

    /**
     * Alternative method: Get line points along the fitted line (similar to Python)
     */
    public List<Point> getLinePoints(LineSegment line, int numPoints) {
        List<Point> points = new ArrayList<>();

        if (line == null) return points;

        double dx = line.end.x - line.start.x;
        double dy = line.end.y - line.start.y;

        for (int i = 0; i < numPoints; i++) {
            double t = (double) i / (numPoints - 1);
            int x = (int) Math.round(line.start.x + t * dx);
            int y = (int) Math.round(line.start.y + t * dy);
            points.add(new Point(x, y));
        }

        return points;
    }

    /**
     * Test method to check color detection
     */
    public Mat getColorMask(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        Mat mask = createRedMask(hsvFrame);
        Mat cleaned = applyMorphologyPythonStyle(mask);

        hsvFrame.release();
        mask.release();

        return cleaned;
    }

    /**
     * Draws detected lines on the frame for visualization
     */
    public void drawDetectedLines(Mat frame, List<LineSegment> lines, Scalar color) {
        for (LineSegment line : lines) {
            // Draw the main line
            Imgproc.line(frame, line.start, line.end, color, 3);

            // Draw center point
            Point center = line.getCenter();
            Imgproc.circle(frame, center, 8, new Scalar(0, 255, 0), -1);

            // Draw endpoints
            Imgproc.circle(frame, line.start, 5, new Scalar(255, 0, 0), -1);
            Imgproc.circle(frame, line.end, 5, new Scalar(0, 0, 255), -1);

            // Add text with line info
            String info = String.format("L:%.1f A:%.1f", line.getLength(), Math.toDegrees(line.getAngle()));
            Imgproc.putText(frame, info, new Point(center.x + 10, center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
        }
    }

    // LineSegment class with improved toString method
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

        @Override
        public String toString() {
            return String.format("LineSegment[%s]: Start(%.1f,%.1f) End(%.1f,%.1f) Length:%.1f Angle:%.1f°",
                    cameraId, start.x, start.y, end.x, end.y, getLength(), Math.toDegrees(getAngle()));
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

        public boolean hasBothCameraDetections() {
            return !leftLines.isEmpty() && !rightLines.isEmpty();
        }
    }

    public void release() {
        if (morphKernel != null) {
            morphKernel.release();
        }
        if (closeKernel != null) {
            closeKernel.release();
        }
    }
}

// Example usage in your FTC OpMode:
/*
public class StereoVisionOpMode extends LinearOpMode {
    private RedLineDetector detector;

    @Override
    public void runOpMode() {
        detector = new RedLineDetector();
        detector.setDebugMode(true);

        waitForStart();

        while (opModeIsActive()) {
            Mat leftFrame = getLeftCameraFrame();
            Mat rightFrame = getRightCameraFrame();

            // Detect red lines (should now return 0-1 lines per camera)
            RedLineDetector.DetectionResult result = detector.detectRedLines(leftFrame, rightFrame);

            // Draw results
            detector.drawDetectedLines(leftFrame, result.leftLines, new Scalar(0, 0, 255));
            detector.drawDetectedLines(rightFrame, result.rightLines, new Scalar(0, 0, 255));

            telemetry.addData("Left Lines", result.leftLines.size());
            telemetry.addData("Right Lines", result.rightLines.size());
            telemetry.addData("Both Cameras", result.hasBothCameraDetections());
            telemetry.update();

            // If both cameras detected a line, proceed with 3D triangulation
            if (result.hasBothCameraDetections()) {
                LineSegment leftLine = result.leftLines.get(0);
                LineSegment rightLine = result.rightLines.get(0);

                // Get corresponding points along the lines
                List<Point> leftPoints = detector.getLinePoints(leftLine, 50);
                List<Point> rightPoints = detector.getLinePoints(rightLine, 50);

                // TODO: Use these points for 3D triangulation
            }
        }

        detector.release();
    }
}
*/