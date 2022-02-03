# Camera IMU Calibration
 An algorithm to find the rotation matrix between the camera and gyroscope:

This tool takes camera files with aruco patterns and IMU data in quaternion format. Extract the rotation matrix from the image
1.Covert your data to image sequence( VideoToImageConvertor.m) (you can provide your own undistorted image)
2.Estimate the pose of each frame (PoseEstimation.py)
3.Run the calibration process (GyroscopeCalibrator.m)