# FTC Stereo Vision Setup Instructions

## Hardware Requirements
- 2 identical webcams (same model recommended)
- FTC Robot Controller phone/Control Hub
- Chessboard calibration pattern (9x6 internal corners)
- Print chessboard on rigid surface (cardboard/foam board)

## Hardware Recommendations
- **Cameras**: Logitech C920, C930e, or similar
- **Baseline**: 150-200mm for FTC field distances
- **Resolution**: 640x480 (good balance of speed/accuracy)
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

### 2. Hardware Configuration
In the Robot Controller app:
1. Go to "Configure Robot"
2. Add both cameras:
    - Name: "leftCamera" (left camera)
    - Name: "rightCamera" (right camera)
3. Save configuration

### 3. Camera Mounting
- Mount cameras rigidly with known baseline distance
- Ensure cameras are level and parallel
- Typical baseline: 10-20cm for FTC field distances
- Both cameras should have similar field of view

## Calibration Process

### Step 1: Print Chessboard
- Download standard 9x6 chessboard pattern
- Print on A4 paper, mount on rigid surface
- Measure actual square size (typically 25mm)
- Update `SQUARE_SIZE` in code if different

### Step 2: Run Calibration
1. Deploy the calibration OpMode
2. Position chessboard in camera view
3. Ensure both cameras can see the entire board
4. Press 'A' to capture frames when board is clearly visible
5. Capture 20+ frames from different angles/distances
6. Press 'B' to perform calibration

### Step 3: Calibration Tips
- Move chessboard to different positions
- Vary distance from cameras
- Tilt board at different angles
- Cover entire camera field of view
- Ensure good lighting, avoid shadows

## Expected Results
- Individual camera RMS error: < 1.0 pixel
- Stereo calibration RMS error: < 1.0 pixel
- Lower RMS = better calibration accuracy

## Next Steps
After successful calibration:
1. Save calibration parameters to file
2. Load parameters in 3D reconstruction code
3. Implement stereo rectification
4. Perform 3D triangulation for object detection

## Common Issues
- **Chessboard not detected**: Improve lighting, ensure board is flat
- **High RMS error**: Capture more frames, improve board positioning
- **Camera sync issues**: Ensure both cameras have same resolution/framerate
- **Distortion**: Check camera mounting, ensure lenses are clean

## Future work
- Improve the single camera calibration with MRCAL instead of OpenCV. MRCAL doesn't have direct Java/Android API support. We can do/manage the single camera calibraion offline with seperate python code.
- Calibration images are currently stored in my Mac local disk. We should handle the real images differently in FTC environment because:
  - •Stored in the assets folder or res/raw and loaded using Android's AssetManager or resource APIs.
  - •Stored on the Robot Controller's internal storage or SD card: In a location accessible by your app (e.g., within the app's private directory or a publicly accessible directory like FIRST/calibrationimages/ on the SD card). You'd then use standard Java File I/O.•Captured live from the camera: This is the most common approach for on-robot calibration.

## Open questions
- stereo calibration (extrinsic parameters - relative position between cameras) needs to be done frequently because Robot vibration/impacts can change camera alignment
- If we have to do stereo calibration during the game, we need the chess board print to do that.
- Is the redline position accurate in the real game? 
    
## File Structure
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

## Result
### left camera calibration result and analysis
{
    "cameraMatrix": [1806.53306139, 0.00000000, 719.50000000, 0.00000000, 1809.20766166, 479.50000000, 0.00000000, 0.00000000, 1.00000000],
    "distortionCoeffs": [-0.38553184, 0.18314920, 0.00000000, 0.00000000, 1.19447628],
    "rms": 0.79670028,
    "imageSize": {
        "width": 1440.0,
        "height": 960.0
        }
}
Camera Matrix Analysis:

fx = 1806.53, fy = 1809.20: These are your focal lengths in pixels. They're very close to each other (good - indicates minimal aspect ratio distortion)
cx = 719.50, cy = 479.50: These are your principal point coordinates (optical center)

cx ≈ width/2 = 1440/2 = 720 ✓
cy ≈ height/2 = 960/2 = 480 ✓
The optical center is almost perfectly centered, which is ideal

Distortion Coefficients:

k1 = -0.386: Moderate barrel distortion (negative = barrel)
k2 = 0.183: Slight pincushion correction at edges
p1 = 0.0, p2 = 0.0: Tangential distortion fixed to zero (as intended)
k3 = 1.194: Higher-order radial distortion correction

Quality Metrics:

RMS = 0.797: This is good! RMS < 1.0 indicates accurate calibration
Image Size: 1440x960 