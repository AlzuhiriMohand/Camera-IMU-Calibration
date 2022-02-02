# Camera IMU Calibration
 An algorith to find the rotation matrix between the camera and gyroscope
you need camera calibration files first:
1.Covert your data to image sequence( VideoToImageConvertor.m)
2.Estimate the pose of each frame (PoseEstimation.py)
3.Run the calibration process (GyroscopeCalibrator.m)