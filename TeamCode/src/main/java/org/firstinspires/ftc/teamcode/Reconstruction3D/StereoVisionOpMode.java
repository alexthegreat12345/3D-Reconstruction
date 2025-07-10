package org.firstinspires.ftc.teamcode.Reconstruction3D;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Stereo Vision Red Line Detection", group="Vision")
public class StereoVisionOpMode extends LinearOpMode {

    private OpenCvCamera leftCamera;
    private OpenCvCamera rightCamera;
    private StereoVision3DCalculator stereoCalculator;

    // Camera calibration matrices (load these from your calibration files)
    private Mat cameraMatrix1, cameraMatrix2;
    private Mat distCoeffs1, distCoeffs2;
    private Mat R, T; // From stereo calibration

    // Current frames
    private Mat leftFrame = new Mat();
    private Mat rightFrame = new Mat();
    private boolean framesReady = false;

    // Red line detection results
    private List<Point> leftRedLinePoints = new ArrayList<>();
    private List<Point> rightRedLinePoints = new ArrayList<>();

    @Override
    public void runOpMode() {
        // Initialize OpenCV
        initializeOpenCV();

        // Load calibration data (implement these methods based on your calibration storage)
        loadCalibrationData();

        // Initialize stereo calculator
        Size imageSize = new Size(640, 480); // Adjust based on your camera resolution
        stereoCalculator = new StereoVision3DCalculator(
                cameraMatrix1, distCoeffs1,
                cameraMatrix2, distCoeffs2,
                R, T, imageSize
        );

        // Start cameras
        startCameras();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (framesReady) {
                processFrames();
            }

            telemetry.update();
            sleep(50); // Process at ~20 FPS
        }

        // Stop cameras
        leftCamera.stopStreaming();
        rightCamera.stopStreaming();
    }

    private void initializeOpenCV() {
        // Initialize OpenCV loader
        if (!OpenCVLoader.initDebug()) {
            telemetry.addData("OpenCV", "Failed to load");
        } else {
            telemetry.addData("OpenCV", "Successfully loaded");
        }
    }

    private void startCameras() {
        // Get camera hardware
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize left camera
        leftCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "leftCamera"), cameraMonitorViewId);

        // Initialize right camera
        rightCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "rightCamera"));

        // Set up pipelines
        leftCamera.setPipeline(new LeftCameraPipeline());
        rightCamera.setPipeline(new RightCameraPipeline());

        // Start streaming
        leftCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                leftCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Left Camera Error", errorCode);
            }
        });

        rightCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                rightCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Right Camera Error", errorCode);
            }
        });
    }

    private void processFrames() {
        // Detect red lines in both frames
        leftRedLinePoints = detectRedLine(leftFrame);
        rightRedLinePoints = detectRedLine(rightFrame);

        telemetry.addData("Left Red Points", leftRedLinePoints.size());
        telemetry.addData("Right Red Points", rightRedLinePoints.size());

        if (!leftRedLinePoints.isEmpty() && !rightRedLinePoints.isEmpty()) {
            // Calculate 3D coordinates using direct triangulation (more efficient)
            List<Point3> redLine3D = stereoCalculator.calculate3DCoordinates(
                    leftFrame, rightFrame, leftRedLinePoints, rightRedLinePoints
            );

            // Alternative: Use disparity-based method (more computationally intensive)
            // List<Point3> redLine3D = stereoCalculator.calculate3DCoordinatesWithDisparity(
            //     leftFrame, rightFrame, leftRedLinePoints, rightRedLinePoints
            // );

            // Filter and refine points
            redLine3D = stereoCalculator.filterAndRefine3DPoints(redLine3D);

            if (!redLine3D.isEmpty()) {
                // Get center point of red line
                Point3 centerPoint = stereoCalculator.getRedLineCenterPoint();

                // Convert to robot coordinates (adjust parameters for your setup)
                Point3 robotCoords = stereoCalculator.convertToRobotCoordinates(
                        centerPoint, 0.2, 15.0 // 20cm camera height, 15° tilt
                );

                // Calculate distance and angle
                double distance = stereoCalculator.getDistanceToRedLine();
                double angle = stereoCalculator.getAngleToRedLine();

                // Display results
                telemetry.addData("3D Points Found", redLine3D.size());
                telemetry.addData("Red Line Center",
                        String.format("X: %.2f, Y: %.2f, Z: %.2f",
                                centerPoint.x, centerPoint.y, centerPoint.z));
                telemetry.addData("Robot Coordinates",
                        String.format("X: %.2f, Y: %.2f, Z: %.2f",
                                robotCoords.x, robotCoords.y, robotCoords.z));
                telemetry.addData("Distance to Line", String.format("%.2f meters", distance));
                telemetry.addData("Angle to Line", String.format("%.1f degrees", angle));

                // Use these values for robot navigation
                navigateToRedLine(robotCoords, distance, angle);
            } else {
                telemetry.addData("Status", "No valid 3D points found");
            }
        } else {
            telemetry.addData("Status", "Red line not detected in both cameras");
        }

        framesReady = false;
    }

    private List<Point> detectRedLine(Mat frame) {
        List<Point> redPoints = new ArrayList<>();

        // Convert BGR to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        // Define red color range in HSV
        Scalar lowerRed1 = new Scalar(0, 50, 50);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 50, 50);
        Scalar upperRed2 = new Scalar(180, 255, 255);

        // Create masks for red color
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();

        Core.inRange(hsv, lowerRed1, upperRed1, mask1);
        Core.inRange(hsv, lowerRed2, upperRed2, mask2);
        Core.bitwise_or(mask1, mask2, mask);

        // Morphological operations to clean up the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Process contours to extract line points
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 100) { // Minimum area threshold
                // Get bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Check if it looks like a line (width > height)
                if (boundingRect.width > boundingRect.height * 2) {
                    // Sample points along the line
                    Point[] contourPoints = contour.toArray();
                    for (int i = 0; i < contourPoints.length; i += 5) { // Sample every 5th point
                        redPoints.add(contourPoints[i]);
                    }
                }
            }
        }

        return redPoints;
    }

    private void navigateToRedLine(Point3 robotCoords, double distance, double angle) {
        // Implement your robot navigation logic here
        // This is where you would command your robot to move towards the red line

        // Example navigation logic:
        if (distance > 0.5) { // If more than 50cm away
            if (Math.abs(angle) > 10) { // If angle is significant
                // Turn towards the line
                telemetry.addData("Action", "Turning " + (angle > 0 ? "right" : "left"));
            } else {
                // Move forward
                telemetry.addData("Action", "Moving forward");
            }
        } else {
            // Close enough to the line
            telemetry.addData("Action", "Red line reached!");
        }
    }

    private void loadCalibrationData() {
        // TODO: Load your calibration matrices from files
        // This is where you would load the calibration data from your previous steps

        // Example initialization (replace with your actual calibration data)
        cameraMatrix1 = Mat.eye(3, 3, CvType.CV_64F);
        cameraMatrix2 = Mat.eye(3, 3, CvType.CV_64F);
        distCoeffs1 = Mat.zeros(1, 5, CvType.CV_64F);
        distCoeffs2 = Mat.zeros(1, 5, CvType.CV_64F);
        R = Mat.eye(3, 3, CvType.CV_64F);
        T = Mat.zeros(3, 1, CvType.CV_64F);

        // You should replace this with code to load from your calibration files
        telemetry.addData("Calibration", "Using default values - replace with actual calibration data");
    }

    // Pipeline for left camera
    class LeftCameraPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(leftFrame);
            framesReady = true;
            return input;
        }
    }

    // Pipeline for right camera
    class RightCameraPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(rightFrame);
            return input;
        }
    }
}