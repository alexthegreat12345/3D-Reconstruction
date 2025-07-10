# FTC Stereo Vision 3D Reconstruction

## Hardware Requirements
- 2 identical webcams (same model recommended)
- FTC Robot Controller phone/Control Hub
- Chessboard calibration pattern (9x6 internal corners recommended)
- Print chessboard on rigid surface (cardboard/foam board)

## Hardware Recommendations
- **Cameras**: Logitech C920, C930e, or similar
- **Baseline**: 150-200mm for FTC field distances (adjust based on your setup)
- **Resolution**: 640x480 (good balance of speed/accuracy, ensure consistency with calibration)
- **Frame rate**: 30 FPS minimum

## Software Setup

### 1. FTC SDK Configuration
Add to your `build.gradle` (Module: TeamCode):
```gradle
dependencies {
    implementation 'org.openftc:easyopencv:1.7.0'
    implementation 'org.opencv:opencv-android:4.5.1'
}
```
*(Note: Ensure your `easyopencv` version is compatible with your FTC SDK version. The `opencv-android` dependency might be implicitly handled by EasyOpenCV, but it's good to be aware of it.)*

### 2. Hardware Configuration
In the Robot Controller app:
1. Go to "Configure Robot"
2. Add both cameras via the configuration editor:
    - Example Name: "leftWebcam"
    - Example Name: "rightWebcam"
3. Save configuration

### 3. Camera Mounting
- Mount cameras rigidly with a known baseline distance (measure accurately).
- Ensure cameras are as level and parallel as possible.
- Typical baseline: 10-20cm for FTC field distances. This is a critical parameter for 3D reconstruction.
- Both cameras should have a similar field of view and be focused appropriately.

## Calibration Process

This process is crucial for accurate 3D reconstruction.

### Print Chessboard
- Use a standard chessboard pattern (e.g., 9x6 internal corners).
- Print it clearly on A4/Letter paper and mount it flat on a rigid surface.
- Measure the actual side length of a single square very precisely (e.g., 25mm). This `SQUARE_SIZE` is vital.
- Update `SQUARE_SIZE_MM` in `SingleCameraCalibration.java` and `StereoCameraCalibration.java` if different.

### Calibration Steps:

#### Step 1: Single Camera Intrinsic Calibration
- **Purpose**: To determine the internal camera parameters (focal length, principal point, distortion coefficients) for *each* camera independently.
- **Process**:
    1. Run an OpMode that uses `SingleCameraCalibration.java`.
    2. Capture 15-20+ images of the chessboard with *each* camera.
        - Vary the chessboard's position, distance, and orientation.
        - Ensure the entire board is visible and well-lit.
        - Avoid motion blur.
    3. The calibration process will save `left_camera_intrinsics.json` and `right_camera_intrinsics.json`.
- **Key Files**: `SingleCameraCalibration.java`

#### Step 2: Stereo Camera Extrinsic Calibration
- **Purpose**: To determine the relative position and orientation (rotation and translation) between the two cameras.
- **Process**:
    1. Run an OpMode that uses `StereoCameraCalibration.java`.
    2. This process requires the intrinsic parameters from Step 1.
    3. Capture 15-20+ *pairs* of images of the chessboard, where the board is visible to *both* cameras simultaneously.
        - Vary the chessboard's position, distance, and orientation in the shared field of view.
    4. The calibration will use the intrinsic data and the image pairs to compute the extrinsic parameters, saving them to `stereo_calibration_parameters.json`.
- **Key Files**: `StereoCameraCalibration.java`

### Calibration Tips
- Move chessboard to different positions and distances.
- Tilt and rotate the board at various angles.
- Cover as much of the camera's field of view as possible over the set of images.
- Ensure good, even lighting; avoid strong shadows or glare.
- A lower RMS error (ideally < 1.0 for single camera, and as low as possible for stereo) indicates a better calibration.

## Object Detection and 3D Reconstruction Pipeline

### Step 3: Object Detection (e.g., finding red lines on the floor)
- **Purpose**: To identify the object(s) of interest (e.g., red tape lines) in the 2D images from one or both cameras.
- **Process**:
    1. Uses color segmentation (e.g., in HSV color space) and contour analysis to find relevant features.
    2. Filters contours based on properties like area, aspect ratio, etc., to isolate the desired objects.
- **Key Files**: `RedLineDetector.java`

### Step 4: 3D Coordinate Calculation (using stereo vision)
- **Purpose**: To calculate the real-world 3D coordinates of the detected objects using the calibrated stereo camera setup.
- **Process**:
    1. **Rectification**: The stereo calibration parameters (R1, R2, P1, P2) are used to undistort and rectify the images from both cameras, making the epipolar lines horizontal. This simplifies disparity calculation.
    2. **Disparity Calculation**:
        - For corresponding points (e.g., points on the detected red line) in the rectified left and right images, calculate the disparity (the difference in their horizontal positions).
        - `StereoVision3DCalculator.java` implements `StereoBM` (Block Matching) for disparity calculation.
    3. **Triangulation**:
        - Using the disparity, the camera intrinsics, and the stereo extrinsics (specifically the baseline and the Q matrix from `stereoRectify`), the 2D image points are triangulated into 3D world coordinates.
        - `StereoVision3DCalculator.java` contains the logic for this, likely using `Calib3d.reprojectImageTo3D` or a manual triangulation formula based on the Q matrix.
- **Key Files**: `StereoVision3DCalculator.java`, `RedLineDetector.java` (to provide the 2D points)

## Current Implementation Details

- **`SingleCameraCalibration.java`**: Handles intrinsic calibration for individual cameras. Saves/loads calibration data as JSON.
- **`StereoCameraCalibration.java`**: Handles extrinsic calibration for the stereo pair, using pre-computed intrinsics. Saves/loads calibration data as JSON.
- **`RedLineDetector.java`**: Implements detection of red lines based on color thresholding in HSV space and contour analysis. It can find line segments and fit lines to them.
- **`StereoVision3DCalculator.java`**:
    - Loads intrinsic and stereo calibration parameters.
    - Performs image rectification.
    - Calculates disparity maps using `StereoBM`.
    - Implements methods to find corresponding points between left and right images (likely using the detected red line features).
    - Contains logic to triangulate 2D image point pairs (with disparity information) into 3D world coordinates.

## File Structure (Current)
```
TeamCode/
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── StereoCameraCalibration.java
│   ├── StereoVision3D.java (next step)
│   └── RedLineDetector.java (final step)
└── calibration_data/
    ├── left_camera_matrix.xml
    ├── right_camera_matrix.xml
    └── stereo_params.xml
```
## Common Issues & Troubleshooting
- **Chessboard not detected (during calibration)**: Improve lighting, ensure the board is flat and fully visible, check `CHESSBOARD_SIZE`.
- **High RMS error (calibration)**: Capture more varied and high-quality calibration frames. Ensure `SQUARE_SIZE` is accurate.
- **Camera sync issues (for stereo capture)**: Ensure both cameras are triggered as simultaneously as possible if capturing live. For calibration, this is less critical if the scene is static between captures for a pair.
- **Distortion not corrected**: Double-check intrinsic calibration; ensure lens surfaces are clean.
- **Inaccurate 3D points**:
    - Verify all calibration steps (intrinsics and extrinsics). RMS error is a good indicator.
    - Ensure the baseline distance in `stereo_calibration_parameters.json` (derived from T vector) is accurate and makes sense for your physical setup.
    - Check the `RedLineDetector` accuracy; incorrect 2D points will lead to incorrect 3D points.
    - Verify disparity map calculation and the triangulation math in `StereoVision3DCalculator.java`.
    - Ensure the Q matrix is correctly used for `reprojectImageTo3D` or manual triangulation.
- **Disparity map issues**: Tune `StereoBM` parameters (`numDisparities`, `blockSize`). `StereoSGBM` (if you switch to it) offers more parameters but is slower.

## Future Work & Considerations
- **Robustness of RedLineDetector**:
    - Improve immunity to changing lighting conditions.
    - Handle partial occlusions or breaks in the line.
- **Performance Optimization**:
    - Optimize image processing steps in `RedLineDetector` and `StereoVision3DCalculator`.
    - Evaluate if `StereoBM` parameters are optimal for speed vs. accuracy. Consider if `StereoSGBM` is needed despite performance cost.
- **Calibration Management**:
    - The current file-based storage of calibration data is good. Ensure paths are correctly managed on the Robot Controller.
    - Consider an OpMode or utility to easily view current calibration status/RMS errors.
- **Single Camera Calibration with MRCAL**:
    - *MRCAL doesn't have direct Java/Android API support. This would likely remain an offline process using Python if pursued. Current OpenCV calibration is standard and effective if done well.*
- **Dynamic Stereo Calibration (Re-alignment)**:
    - *This is a complex topic. If robot vibration significantly misaligns cameras, a quick re-alignment check using known field features (e.g., AprilTags if available and static) during an init phase might be explored, but a full re-calibration is too slow.*
- **Coordinate System**: Be very clear about which coordinate system the final 3D points are in (e.g., relative to the left camera, or a robot-centric coordinate system). Define this and document it.

## Open Questions
- **Stereo Calibration Frequency**: *Robot vibration/impacts can change camera alignment. The need for re-calibration depends on robot construction and the impacts it sustains. Minor shifts might be tolerable; major shifts will require re-calibration.*
- **On-the-fly Stereo Calibration**: *Full stereo calibration with a chessboard during a match is impractical. Quick re-alignment using field markers is theoretically possible but challenging to implement robustly.*
- **Red Line Position Accuracy in Game**: *This needs to be empirically tested. Accuracy will depend on: calibration quality, red line detection robustness, lighting, camera resolution, and distance to the line.*

## Calibration Results (Examples - Update with your latest)

### Left Camera Calibration Result (Example)
#### Camera Matrix Analysis:

**fx = 1806.53, fy = 1809.20:** These are the camera focal lengths in pixels. They're very close to each other (good - indicates minimal aspect ratio distortion)
**cx = 719.50, cy = 479.50:** These are your principal point coordinates (optical center)

**cx ≈ width/2 = 1440/2 = 720 ✓**
**cy ≈ height/2 = 960/2 = 480 ✓**
The optical center is almost perfectly centered, which is ideal

#### Distortion Coefficients:

**k1 = -0.386:** Moderate barrel distortion (negative = barrel)
**k2 = 0.183:** Slight pincushion correction at edges
**p1 = 0.0, p2 = 0.0:** Tangential distortion fixed to zero (as intended)
**k3 = 1.194:** Higher-order radial distortion correction

#### Quality Metrics:

**RMS = 0.797:** This is good! RMS < 1.0 indicates accurate calibration
Image Size: 1440x960

### Stereo Camera Calibration Result (Example)
#### Stereo Calibration Analysis:
```json
{
  "R": [0.91891195, -0.30592921, 0.24901436, 0.38275524, 0.84417552, -0.37532135, -0.09539006, 0.44019882, 0.89281898],
  "T": [-485.95790648, 317.12975431, 165.49815742],
  "R1": [0.56503958, -0.81076826, 0.15292189, 0.82503804, 0.55669649, -0.09695999, -0.00651900, 0.18095261, 0.98347021],
  "R2": [0.80533906, -0.52555371, -0.27426682, 0.56368316, 0.82212834, 0.07978899, 0.18354913, -0.21885678, 0.95833785],
  "P1": [1797.20210674, 0.00000000, 867.19686508, 0.00000000, 0.00000000, 1797.20210674, 503.92209053, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000],
  "P2": [1797.20210674, 0.00000000, 867.19686508, -1084468.15935737, 0.00000000, 1797.20210674, 503.92209053, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000],
  "Q": [1.00000000, 0.00000000, 0.00000000, -867.19686508, 0.00000000, 1.00000000, 0.00000000, -503.92209053, 0.00000000, 0.00000000, 0.00000000, 1797.20210674, 0.00000000, 0.00000000, 0.00165722, -0.00000000],
  "rms": 5.65665808,
  "imageSize": {
    "width": 1440.0,
    "height": 960.0
  }
}
```


Rotation Matrix (R) - Camera Orientation. This represents the rotation between your left and right cameras.
Translation Vector (T) - Camera Position.
Baseline (primary X component of T, check units, e.g., mm): `T[0]` = -485.96mm suggests the right camera is ~486mm to the *left* of the left camera if the left camera is the origin, or this value needs to be interpreted carefully based on your coordinate system definition. Typically, for a baseline `Tx`, `P2[0][3]` would be `-fx * Tx`. Given `P2[0][3]` is a large negative number and `P1[0][3]` is 0, this `T[0]` indicates the horizontal shift.
Vertical offset: `T[1]` = 317.13mm
Forward/backward offset: `T[2]` = 165.50mm

Rectification Matrices (R1, R2): These transform the cameras to a "rectified" coordinate system where epipolar lines are horizontal.
Projection Matrices (P1, P2): Used for projecting 3D points into the rectified image planes. Note `P2[0,3]` contains the `fx * baseline` term.
Disparity-to-Depth Matrix (Q): Used by `reprojectImageTo3D` to convert a disparity value (and pixel coordinates) to a 3D point.

