# Camera IMU Calibration
 An algorithm to find the rotation matrix between the camera and gyroscope:

This tool takes camera files with aruco patterns and IMU data in quaternion format. 
1.Covert your data to image sequence( VideoToImageConvertor.m) (you can provide your own undistorted image)
2.Estimate the pose of each frame (PoseEstimation.py) (Extract the rotation vectors from the images (camera matrix needed))
3.Run the calibration process (GyroscopeCalibrator.m) 

The core of this repo depends on handEye.m which is an implementation of a paper by Tsai et. al (R.Tsai, R.K.Lenz "A new Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye calibration", IEEE trans. on robotics and Automaion, Vol.5, No.3, June 1989).

The rest of the code is a part of an unpublished work (A link will be provided once the work is published)
