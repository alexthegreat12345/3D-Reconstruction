package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

public class RedLineDetector {

    // More restrictive HSV color ranges based on Python working code
    private static final Scalar LOWER_RED_1 = new Scalar(0, 100, 100);     // Higher saturation/value
    private static final Scalar UPPER_RED_1 = new Scalar(8, 255, 255);     // Narrower hue range
    private static final Scalar LOWER_RED_2 = new Scalar(172, 100, 100);   // Higher saturation/value
    private static final Scalar UPPER_RED_2 = new Scalar(180, 255, 255);

    // Improved line detection parameters
    private static final double MIN_LINE_LENGTH = 150;   // Increased
    private static final double MAX_LINE_GAP = 5;        // Decreased
    private static final double RHO = 1;
    private static final double THETA = Math.PI / 180;
    private static final int THRESHOLD = 80;             // Increased for fewer false positives

    // Morphological operation kernels
    private Mat morphKernel;
    private Mat closeKernel;
    private Mat smallKernel;

    private boolean debugMode = false;

    public RedLineDetector() {
        // Kernels for morphological operations
        morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        closeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        smallKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 2));
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
     * Improved line detection using HoughLinesP directly on the mask
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

        // Create red mask
        Mat redMask = createRedMask(hsvFrame);

        if (debugMode) {
            int nonZeroPixels = Core.countNonZero(redMask);
            System.out.println("Non-zero pixels in red mask: " + nonZeroPixels);
        }

        // Apply lighter morphological operations to preserve line structure
        Mat cleanedMask = applyLightMorphology(redMask);

        // Apply Gaussian blur to reduce noise before edge detection
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(cleanedMask, blurred, new Size(3, 3), 0);

        // Method 1: Use HoughLinesP directly on the cleaned mask
        LineSegment houghLine = detectLineWithHough(blurred, cameraId, frame.size());

        if (houghLine != null) {
            result.add(houghLine);
            if (debugMode) {
                System.out.println("Hough line detected: " + houghLine.toString());
            }
        } else {
            // Method 2: Fallback to skeleton-based approach
            LineSegment skeletonLine = extractLineFromSkeleton(cleanedMask, cameraId, frame.size());
            if (skeletonLine != null) {
                result.add(skeletonLine);
                if (debugMode) {
                    System.out.println("Skeleton line detected: " + skeletonLine.toString());
                }
            } else {
                // Method 3: Final fallback to contour-based approach
                List<LineSegment> contourLines = detectBestLineInFrameContour(cleanedMask, cameraId, frame.size());
                if (!contourLines.isEmpty()) {
                    result.addAll(contourLines);
                    if (debugMode) {
                        System.out.println("Contour line detected: " + contourLines.get(0).toString());
                    }
                }
            }
        }

        // Cleanup
        hsvFrame.release();
        redMask.release();
        cleanedMask.release();
        blurred.release();

        return result;
    }

    /**
     * Detect line using HoughLinesP
     */
    private LineSegment detectLineWithHough(Mat mask, String cameraId, Size frameSize) {
        Mat lines = new Mat();
        Imgproc.HoughLinesP(mask, lines, RHO, THETA, THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);

        if (debugMode) {
            System.out.println("HoughLinesP detected " + lines.rows() + " lines");
        }

        if (lines.rows() == 0) {
            lines.release();
            return null;
        }

        // Convert detected lines to LineSegment objects
        List<LineSegment> detectedLines = new ArrayList<>();
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            Point start = new Point(line[0], line[1]);
            Point end = new Point(line[2], line[3]);
            detectedLines.add(new LineSegment(start, end, cameraId));
        }

        lines.release();

        // Filter and select the best line
        return selectBestLine(detectedLines, frameSize);
    }

    /**
     * Lighter morphological operations that preserve line structure
     */
    private Mat applyLightMorphology(Mat mask) {
        Mat result = new Mat();

        // Only apply closing to connect small gaps, skip opening to preserve thin lines
        Imgproc.morphologyEx(mask, result, Imgproc.MORPH_CLOSE, smallKernel);

        return result;
    }

    /**
     * Select the best line based on length and position
     */
    private LineSegment selectBestLine(List<LineSegment> lines, Size frameSize) {
        if (lines.isEmpty()) return null;

        // Sort by length (longest first)
        lines.sort((a, b) -> Double.compare(b.getLength(), a.getLength()));

        // Additional filtering criteria
        List<LineSegment> filteredLines = new ArrayList<>();
        for (LineSegment line : lines) {
            // Filter out lines that are too short or too close to image edges
            if (line.getLength() > MIN_LINE_LENGTH &&
                    isLineReasonable(line, frameSize)) {
                filteredLines.add(line);
            }
        }

        return filteredLines.isEmpty() ? null : filteredLines.get(0);
    }

    /**
     * Check if a line is reasonable (not too close to edges, reasonable angle)
     */
    private boolean isLineReasonable(LineSegment line, Size frameSize) {
        // Check if line is too close to image edges
        double edgeMargin = 10;
        if (line.start.x < edgeMargin || line.start.x > frameSize.width - edgeMargin ||
                line.start.y < edgeMargin || line.start.y > frameSize.height - edgeMargin ||
                line.end.x < edgeMargin || line.end.x > frameSize.width - edgeMargin ||
                line.end.y < edgeMargin || line.end.y > frameSize.height - edgeMargin) {
            return false;
        }

        // Check angle - prefer lines that are not perfectly horizontal/vertical unless they're very long
        double angle = Math.abs(Math.toDegrees(line.getAngle()));
        if (Math.abs(angle) < 5 || Math.abs(angle - 90) < 5) {
            return line.getLength() > MIN_LINE_LENGTH * 2;
        }

        return true;
    }

    /**
     * Alternative approach: Use skeleton/thinning to get line centerline
     */
    private LineSegment extractLineFromSkeleton(Mat mask, String cameraId, Size frameSize) {
        // Apply morphological thinning to get skeleton
        Mat skeleton = thinning(mask);

        // Find endpoints of the skeleton
        List<Point> endPoints = findSkeletonEndpoints(skeleton);

        if (debugMode) {
            System.out.println("Skeleton endpoints found: " + endPoints.size());
        }

        skeleton.release();

        if (endPoints.size() >= 2) {
            // Find the most distant pair of endpoints
            Point start = endPoints.get(0);
            Point end = endPoints.get(1);
            double maxDist = 0;

            for (int i = 0; i < endPoints.size(); i++) {
                for (int j = i + 1; j < endPoints.size(); j++) {
                    double dist = Math.sqrt(Math.pow(endPoints.get(i).x - endPoints.get(j).x, 2) +
                            Math.pow(endPoints.get(i).y - endPoints.get(j).y, 2));
                    if (dist > maxDist) {
                        maxDist = dist;
                        start = endPoints.get(i);
                        end = endPoints.get(j);
                    }
                }
            }

            if (maxDist > MIN_LINE_LENGTH) {
                return new LineSegment(start, end, cameraId);
            }
        }

        return null;
    }

    /**
     * Simple thinning algorithm (Zhang-Suen)
     */
    private Mat thinning(Mat src) {
        Mat dst = src.clone();
        Mat prev = new Mat();
        Mat diff = new Mat();

        int iterations = 0;
        int maxIterations = 50; // Prevent infinite loops

        do {
            prev = dst.clone();
            thinningIteration(dst, 0);
            thinningIteration(dst, 1);
            Core.absdiff(dst, prev, diff);
            prev.release();
            iterations++;
        } while (Core.countNonZero(diff) > 0 && iterations < maxIterations);

        diff.release();
        return dst;
    }

    /**
     * One iteration of Zhang-Suen thinning
     */
    private void thinningIteration(Mat img, int iter) {
        Mat marker = Mat.zeros(img.size(), CvType.CV_8UC1);

        for (int i = 1; i < img.rows() - 1; i++) {
            for (int j = 1; j < img.cols() - 1; j++) {
                double[] pixel = img.get(i, j);
                if (pixel[0] == 0) continue;

                // Get 3x3 neighborhood
                int[] p = new int[9];
                p[0] = (int)img.get(i-1, j-1)[0];
                p[1] = (int)img.get(i-1, j)[0];
                p[2] = (int)img.get(i-1, j+1)[0];
                p[3] = (int)img.get(i, j+1)[0];
                p[4] = (int)img.get(i+1, j+1)[0];
                p[5] = (int)img.get(i+1, j)[0];
                p[6] = (int)img.get(i+1, j-1)[0];
                p[7] = (int)img.get(i, j-1)[0];
                p[8] = p[0]; // wrap around

                // Count transitions
                int transitions = 0;
                for (int k = 0; k < 8; k++) {
                    if (p[k] == 0 && p[k+1] > 0) transitions++;
                }

                // Count non-zero neighbors
                int nonZeroCount = 0;
                for (int k = 0; k < 8; k++) {
                    if (p[k] > 0) nonZeroCount++;
                }

                // Apply Zhang-Suen conditions
                boolean condition1 = (nonZeroCount >= 2 && nonZeroCount <= 6);
                boolean condition2 = (transitions == 1);
                boolean condition3, condition4;

                if (iter == 0) {
                    condition3 = (p[1] == 0 || p[3] == 0 || p[5] == 0);
                    condition4 = (p[3] == 0 || p[5] == 0 || p[7] == 0);
                } else {
                    condition3 = (p[1] == 0 || p[3] == 0 || p[7] == 0);
                    condition4 = (p[1] == 0 || p[5] == 0 || p[7] == 0);
                }

                if (condition1 && condition2 && condition3 && condition4) {
                    marker.put(i, j, 255);
                }
            }
        }

        // Remove marked pixels
        Core.bitwise_xor(img, marker, img);
        marker.release();
    }

    /**
     * Find endpoints in skeleton (pixels with only one neighbor)
     */
    private List<Point> findSkeletonEndpoints(Mat skeleton) {
        List<Point> endpoints = new ArrayList<>();

        for (int i = 1; i < skeleton.rows() - 1; i++) {
            for (int j = 1; j < skeleton.cols() - 1; j++) {
                double[] pixel = skeleton.get(i, j);
                if (pixel[0] == 0) continue;

                // Count non-zero neighbors
                int neighbors = 0;
                for (int di = -1; di <= 1; di++) {
                    for (int dj = -1; dj <= 1; dj++) {
                        if (di == 0 && dj == 0) continue;
                        if (skeleton.get(i + di, j + dj)[0] > 0) neighbors++;
                    }
                }

                // Endpoint has only 1 neighbor
                if (neighbors == 1) {
                    endpoints.add(new Point(j, i));
                }
            }
        }

        return endpoints;
    }

    /**
     * Original contour-based detection as fallback
     */
    private List<LineSegment> detectBestLineInFrameContour(Mat mask, String cameraId, Size frameSize) {
        List<LineSegment> result = new ArrayList<>();

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (debugMode) {
            System.out.println("Found " + contours.size() + " contours");
        }

        // Filter contours by area and aspect ratio
        List<MatOfPoint> lineContours = filterLineContours(contours);

        if (debugMode) {
            System.out.println("Detected " + lineContours.size() + " potential line contours");
        }

        // Find the largest contour and fit a line through it
        if (!lineContours.isEmpty()) {
            MatOfPoint largestContour = findLargestContour(lineContours);
            LineSegment bestLine = fitLineToContour(largestContour, cameraId, frameSize);

            if (bestLine != null) {
                result.add(bestLine);
                if (debugMode) {
                    System.out.println("Best line found: " + bestLine.toString());
                }
            }
        }

        // Clean up
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

            if (area > 100) {  // Minimum area threshold
                Rect boundingRect = Imgproc.boundingRect(contour);
                double aspectRatio = Math.max(boundingRect.width, boundingRect.height) /
                        (double) Math.min(boundingRect.width, boundingRect.height);

                if (debugMode) {
                    System.out.println("Contour area: " + area + ", Aspect ratio: " + String.format("%.2f", aspectRatio));
                }

                // Check if it's line-like (high aspect ratio)
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
     * Robust fit line implementation with fallback methods
     */
    private LineSegment fitLineToContour(MatOfPoint contour, String cameraId, Size frameSize) {
        Point[] contourPoints = contour.toArray();

        if (contourPoints.length < 10) {
            if (debugMode) {
                System.out.println("Not enough points to fit a line: " + contourPoints.length);
            }
            return null;
        }

        if (debugMode) {
            System.out.println("Fitting line to " + contourPoints.length + " points");
        }

        // Method 1: Try OpenCV fitLine with MatOfPoint2f
        try {
            List<Point> pointsList = new ArrayList<>();
            for (Point p : contourPoints) {
                pointsList.add(p);
            }

            MatOfPoint2f points2f = new MatOfPoint2f();
            points2f.fromList(pointsList);

            Mat lineMat = new Mat();
            Imgproc.fitLine(points2f, lineMat, Imgproc.DIST_L2, 0, 0.01, 0.01);

            if (debugMode) {
                System.out.println("FitLine result total elements: " + lineMat.total());
            }

            if (lineMat.total() == 4) {
                double[] lineParams = new double[4];
                lineMat.get(0, 0, lineParams);

                if (debugMode) {
                    System.out.printf("Line parameters: vx=%.4f, vy=%.4f, x0=%.4f, y0=%.4f%n",
                            lineParams[0], lineParams[1], lineParams[2], lineParams[3]);
                }

                double vx = lineParams[0];
                double vy = lineParams[1];
                double x0 = lineParams[2];
                double y0 = lineParams[3];

                // Calculate endpoints
                Point startPoint = null;
                Point endPoint = null;

                if (Math.abs(vx) > 1e-6) {
                    double tLeft = -x0 / vx;
                    double tRight = (frameSize.width - 1 - x0) / vx;

                    int leftY = (int) Math.round(y0 + tLeft * vy);
                    int rightY = (int) Math.round(y0 + tRight * vy);

                    leftY = Math.max(0, Math.min((int) frameSize.height - 1, leftY));
                    rightY = Math.max(0, Math.min((int) frameSize.height - 1, rightY));

                    startPoint = new Point(0, leftY);
                    endPoint = new Point(frameSize.width - 1, rightY);
                } else {
                    int x = (int) Math.round(x0);
                    if (x >= 0 && x < frameSize.width) {
                        startPoint = new Point(x, 0);
                        endPoint = new Point(x, frameSize.height - 1);
                    }
                }

                points2f.release();
                lineMat.release();

                if (startPoint != null && endPoint != null) {
                    LineSegment result = new LineSegment(startPoint, endPoint, cameraId);
                    if (debugMode) {
                        System.out.println("Successfully fitted line using OpenCV: " + result.toString());
                    }
                    return result;
                }
            }

            points2f.release();
            lineMat.release();

        } catch (Exception e) {
            if (debugMode) {
                System.err.println("OpenCV fitLine failed, trying alternative method: " + e.getMessage());
            }
        }

        // Method 2: Fallback to bounding rectangle approach
        try {
            Rect boundingRect = Imgproc.boundingRect(contour);

            if (debugMode) {
                System.out.println("Using bounding rectangle fallback. Rect: " + boundingRect.toString());
            }

            // Determine if it's more horizontal or vertical
            boolean isHorizontal = boundingRect.width > boundingRect.height;

            Point startPoint, endPoint;

            if (isHorizontal) {
                // Create horizontal line through center of bounding rect
                int centerY = boundingRect.y + boundingRect.height / 2;
                startPoint = new Point(0, centerY);
                endPoint = new Point(frameSize.width - 1, centerY);
            } else {
                // Create vertical line through center of bounding rect
                int centerX = boundingRect.x + boundingRect.width / 2;
                startPoint = new Point(centerX, 0);
                endPoint = new Point(centerX, frameSize.height - 1);
            }

            LineSegment result = new LineSegment(startPoint, endPoint, cameraId);
            if (debugMode) {
                System.out.println("Successfully fitted line using bounding rectangle: " + result.toString());
            }
            return result;

        } catch (Exception e) {
            if (debugMode) {
                System.err.println("Bounding rectangle fallback also failed: " + e.getMessage());
            }
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
        Mat cleaned = applyLightMorphology(mask);

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
        if (smallKernel != null) {
            smallKernel.release();
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