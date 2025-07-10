package org.firstinspires.ftc.teamcode.Reconstruction3D;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;
import java.util.ArrayList;
import java.util.List;

public class StereoVision3DCalculator {

    // Camera calibration parameters (from your previous steps)
    private Mat cameraMatrix1, cameraMatrix2;
    private Mat distCoeffs1, distCoeffs2;

    // Stereo calibration parameters
    private Mat R, T, E, F;  // Rotation, Translation, Essential, Fundamental matrices
    private Mat R1, R2, P1, P2, Q;  // Rectification matrices
    private Mat map1x, map1y, map2x, map2y;  // Rectification maps

    // Image dimensions
    private Size imageSize;

    // 3D coordinates storage
    private List<Point3> redLine3DPoints;

    public StereoVision3DCalculator(Mat cameraMatrix1, Mat distCoeffs1,
                                    Mat cameraMatrix2, Mat distCoeffs2,
                                    Mat R, Mat T, Size imageSize) {
        this.cameraMatrix1 = cameraMatrix1;
        this.distCoeffs1 = distCoeffs1;
        this.cameraMatrix2 = cameraMatrix2;
        this.distCoeffs2 = distCoeffs2;
        this.R = R;
        this.T = T;
        this.imageSize = imageSize;
        this.redLine3DPoints = new ArrayList<>();

        initializeStereoRectification();
    }

    /**
     * Initialize stereo rectification maps
     */
    private void initializeStereoRectification() {
        // Initialize matrices
        R1 = new Mat();
        R2 = new Mat();
        P1 = new Mat();
        P2 = new Mat();
        Q = new Mat();

        // Compute rectification transforms
        Calib3d.stereoRectify(
                cameraMatrix1, distCoeffs1,
                cameraMatrix2, distCoeffs2,
                imageSize, R, T,
                R1, R2, P1, P2, Q,
                Calib3d.CALIB_ZERO_DISPARITY, -1, imageSize
        );

        // Initialize rectification maps
        map1x = new Mat();
        map1y = new Mat();
        map2x = new Mat();
        map2y = new Mat();

        Calib3d.initUndistortRectifyMap(
                cameraMatrix1, distCoeffs1, R1, P1, imageSize, CvType.CV_32FC1, map1x, map1y
        );

        Calib3d.initUndistortRectifyMap(
                cameraMatrix2, distCoeffs2, R2, P2, imageSize, CvType.CV_32FC1, map2x, map2y
        );
    }

    /**
     * Main method to calculate 3D coordinates of red lines
     * Alternative approach using direct point matching (more efficient for FTC)
     */
    public List<Point3> calculate3DCoordinates(Mat leftImage, Mat rightImage,
                                               List<Point> leftRedLinePoints,
                                               List<Point> rightRedLinePoints) {

        // Step 1: Rectify the stereo images
        Mat leftRectified = new Mat();
        Mat rightRectified = new Mat();

        Imgproc.remap(leftImage, leftRectified, map1x, map1y, Imgproc.INTER_LINEAR);
        Imgproc.remap(rightImage, rightRectified, map2x, map2y, Imgproc.INTER_LINEAR);

        // Step 2: Match red line points between left and right images
        List<Point> matchedLeftPoints = new ArrayList<>();
        List<Point> matchedRightPoints = new ArrayList<>();

        matchRedLinePoints(leftRedLinePoints, rightRedLinePoints,
                matchedLeftPoints, matchedRightPoints);

        // Step 3: Calculate 3D coordinates using direct triangulation
        redLine3DPoints.clear();
        for (int i = 0; i < matchedLeftPoints.size(); i++) {
            Point3 point3D = triangulatePointDirect(matchedLeftPoints.get(i),
                    matchedRightPoints.get(i));
            if (point3D != null) {
                redLine3DPoints.add(point3D);
            }
        }

        return redLine3DPoints;
    }

//    /**
//     * Alternative main method using disparity map (more computationally intensive)
//     */
//    public List<Point3> calculate3DCoordinatesWithDisparity(Mat leftImage, Mat rightImage,
//                                                            List<Point> leftRedLinePoints,
//                                                            List<Point> rightRedLinePoints) {
//
//        // Step 1: Rectify the stereo images
//        Mat leftRectified = new Mat();
//        Mat rightRectified = new Mat();
//
//        Imgproc.remap(leftImage, leftRectified, map1x, map1y, Imgproc.INTER_LINEAR);
//        Imgproc.remap(rightImage, rightRectified, map2x, map2y, Imgproc.INTER_LINEAR);
//
//        // Step 2: Compute disparity map (optional, for visualization)
//        Mat disparity = computeDisparityMap(leftRectified, rightRectified);
//
//        // Step 3: Match red line points between left and right images
//        List<Point> matchedLeftPoints = new ArrayList<>();
//        List<Point> matchedRightPoints = new ArrayList<>();
//
//        matchRedLinePoints(leftRedLinePoints, rightRedLinePoints,
//                matchedLeftPoints, matchedRightPoints);
//
//        // Step 4: Calculate 3D coordinates for matched points
//        redLine3DPoints.clear();
//        for (int i = 0; i < matchedLeftPoints.size(); i++) {
//            Point3 point3D = triangulatePoint(matchedLeftPoints.get(i),
//                    matchedRightPoints.get(i),
//                    disparity);
//            if (point3D != null) {
//                redLine3DPoints.add(point3D);
//            }
//        }
//
//        return redLine3DPoints;
//    }

    /**
     * Compute disparity map using block matching
     */
    private Mat computeDisparityMap(Mat leftRectified, Mat rightRectified) {
        Mat leftGray = new Mat();
        Mat rightGray = new Mat();

        // Convert to grayscale if needed
        if (leftRectified.channels() > 1) {
            Imgproc.cvtColor(leftRectified, leftGray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.cvtColor(rightRectified, rightGray, Imgproc.COLOR_BGR2GRAY);
        } else {
            leftGray = leftRectified;
            rightGray = rightRectified;
        }

        // Parameters for block matching
        int minDisparity = 0;
        int numDisparities = 64; // Must be divisible by 16
        int blockSize = 15; // Odd number, typically 5-21

        Mat disparity = new Mat();

        // Use manual block matching since StereoBM_create is not available
        disparity = computeBlockMatching(leftGray, rightGray, minDisparity, numDisparities, blockSize);

        return disparity;
    }

    /**
     * Manual block matching implementation for disparity calculation
     */
    private Mat computeBlockMatching(Mat leftGray, Mat rightGray,
                                     int minDisparity, int numDisparities, int blockSize) {

        int rows = leftGray.rows();
        int cols = leftGray.cols();
        Mat disparity = Mat.zeros(rows, cols, CvType.CV_8U);

        int halfBlock = blockSize / 2;

        // Process each pixel
        for (int y = halfBlock; y < rows - halfBlock; y++) {
            for (int x = halfBlock; x < cols - halfBlock; x++) {

                double bestDisparity = 0;
                double minSSD = Double.MAX_VALUE;

                // Search for best match within disparity range
                for (int d = minDisparity; d < numDisparities && x - d >= halfBlock; d++) {
                    double ssd = 0;

                    // Calculate Sum of Squared Differences (SSD) for the block
                    for (int by = -halfBlock; by <= halfBlock; by++) {
                        for (int bx = -halfBlock; bx <= halfBlock; bx++) {
                            double leftPixel = leftGray.get(y + by, x + bx)[0];
                            double rightPixel = rightGray.get(y + by, x - d + bx)[0];
                            double diff = leftPixel - rightPixel;
                            ssd += diff * diff;
                        }
                    }

                    // Keep track of best match
                    if (ssd < minSSD) {
                        minSSD = ssd;
                        bestDisparity = d;
                    }
                }

                // Set disparity value (scale to 0-255 range)
                disparity.put(y, x, bestDisparity * 255.0 / numDisparities);
            }
        }

        return disparity;
    }

    /**
     * Match red line points between left and right images using epipolar constraints
     */
    private void matchRedLinePoints(List<Point> leftPoints, List<Point> rightPoints,
                                    List<Point> matchedLeft, List<Point> matchedRight) {

        double maxDistance = 10.0; // Maximum distance for point matching

        for (Point leftPoint : leftPoints) {
            Point bestMatch = null;
            double minDistance = Double.MAX_VALUE;

            // For rectified images, corresponding points should be on the same horizontal line
            for (Point rightPoint : rightPoints) {
                // Check if points are on approximately the same horizontal line
                double verticalDiff = Math.abs(leftPoint.y - rightPoint.y);
                if (verticalDiff > 5.0) continue; // Skip if not on same epipolar line

                // Calculate distance
                double distance = Math.abs(leftPoint.x - rightPoint.x);

                if (distance < minDistance && distance < maxDistance) {
                    minDistance = distance;
                    bestMatch = rightPoint;
                }
            }

            if (bestMatch != null) {
                matchedLeft.add(leftPoint);
                matchedRight.add(bestMatch);
            }
        }
    }

    /**
     * Direct triangulation method (more efficient for FTC)
     */
    private Point3 triangulatePointDirect(Point leftPoint, Point rightPoint) {
        try {
            // Create matrices for triangulation
            Mat projPoints1 = new Mat(1, 1, CvType.CV_32FC2);
            Mat projPoints2 = new Mat(1, 1, CvType.CV_32FC2);

            projPoints1.put(0, 0, leftPoint.x, leftPoint.y);
            projPoints2.put(0, 0, rightPoint.x, rightPoint.y);

            Mat points4D = new Mat();

            // Triangulate using OpenCV's triangulatePoints
            Calib3d.triangulatePoints(P1, P2, projPoints1, projPoints2, points4D);

            // Convert from homogeneous coordinates
            double[] point4D_data = new double[4];
            points4D.get(0, 0, point4D_data);

            if (Math.abs(point4D_data[3]) < 1e-7) {
                return null; // Invalid point
            }

            double x = point4D_data[0] / point4D_data[3];
            double y = point4D_data[1] / point4D_data[3];
            double z = point4D_data[2] / point4D_data[3];

            return new Point3(x, y, z);

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Alternative triangulation using disparity and Q matrix
     */
    private Point3 triangulatePointWithDisparity(Point leftPoint, Point rightPoint, Mat disparity) {
        try {
            // Calculate disparity value
            double disparityValue = leftPoint.x - rightPoint.x;

            if (disparityValue <= 0) {
                return null; // Invalid disparity
            }

            // Create 4D point using Q matrix
            Mat point4D = new Mat(4, 1, CvType.CV_64F);
            point4D.put(0, 0, leftPoint.x);
            point4D.put(1, 0, leftPoint.y);
            point4D.put(2, 0, disparityValue);
            point4D.put(3, 0, 1.0);

            // Transform using Q matrix
            Mat point3D_homo = new Mat();
            Core.gemm(Q, point4D, 1.0, new Mat(), 0.0, point3D_homo);

            // Convert from homogeneous to 3D coordinates
            double[] point3D_data = point3D_homo.get(0, 0);
            double w = point3D_data[3];

            if (Math.abs(w) < 1e-7) {
                return null; // Invalid point
            }

            double x = point3D_data[0] / w;
            double y = point3D_data[1] / w;
            double z = point3D_data[2] / w;

            return new Point3(x, y, z);

        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Filter and refine 3D points (remove outliers)
     */
    public List<Point3> filterAndRefine3DPoints(List<Point3> points3D) {
        if (points3D.size() < 3) return points3D;

        List<Point3> filteredPoints = new ArrayList<>();

        // Calculate median Z value for outlier detection
        List<Double> zValues = new ArrayList<>();
        for (Point3 point : points3D) {
            zValues.add(point.z);
        }
        zValues.sort(Double::compareTo);
        double medianZ = zValues.get(zValues.size() / 2);

        // Filter points based on Z value (depth)
        double maxZDeviation = medianZ * 0.5; // 50% deviation threshold

        for (Point3 point : points3D) {
            // Filter out points too close or too far
            if (point.z > 0.1 && point.z < 5.0 &&
                    Math.abs(point.z - medianZ) < maxZDeviation) {
                filteredPoints.add(point);
            }
        }

        return filteredPoints;
    }

    /**
     * Get the center point of the red line in 3D space
     */
    public Point3 getRedLineCenterPoint() {
        if (redLine3DPoints.isEmpty()) return null;

        double sumX = 0, sumY = 0, sumZ = 0;
        for (Point3 point : redLine3DPoints) {
            sumX += point.x;
            sumY += point.y;
            sumZ += point.z;
        }

        int count = redLine3DPoints.size();
        return new Point3(sumX / count, sumY / count, sumZ / count);
    }

    /**
     * Calculate distance to red line center
     */
    public double getDistanceToRedLine() {
        Point3 center = getRedLineCenterPoint();
        if (center == null) return -1;

        return Math.sqrt(center.x * center.x + center.y * center.y + center.z * center.z);
    }

    /**
     * Get angle to red line (useful for robot navigation)
     */
    public double getAngleToRedLine() {
        Point3 center = getRedLineCenterPoint();
        if (center == null) return 0;

        return Math.atan2(center.x, center.z) * 180.0 / Math.PI;
    }

    /**
     * Convert 3D coordinates to robot coordinate system
     * Assumes camera is mounted on robot with known offset
     */
    public Point3 convertToRobotCoordinates(Point3 cameraPoint,
                                            double cameraHeight,
                                            double cameraTiltAngle) {
        // Apply camera tilt correction
        double tiltRad = Math.toRadians(cameraTiltAngle);
        double correctedY = cameraPoint.y * Math.cos(tiltRad) + cameraPoint.z * Math.sin(tiltRad);
        double correctedZ = -cameraPoint.y * Math.sin(tiltRad) + cameraPoint.z * Math.cos(tiltRad);

        // Translate to robot coordinate system
        return new Point3(
                cameraPoint.x,           // X remains the same (left-right)
                correctedY - cameraHeight, // Y adjusted for camera height
                correctedZ               // Z is forward direction
        );
    }
}