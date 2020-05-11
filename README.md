## RAIN-ROS
ROS end scripts for RAIN application

Build [RAIN](https://github.com/DreVinciBot/RAIN-AR_vision_calibration) on an android device.

Connect Astra Camera, RAIN sends the topic `/camera/depth_registered/points` to the andriod device through a WebSocket connection.

### Required
[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

[ros_astra_camera](https://github.com/orbbec/ros_astra_camera)

[Target Image](https://github.com/DreVinciBot/RAIN-AR_vision_calibration/blob/master/RAIN/Assets/Editor/Vuforia/ImageTargetTextures/RAIN/RainTargetImage_scaled.jpg)

### How to Run
1. Run the command `roslaunch ros_rain rain.launch`. This will open Rviz and display the Astra camera output.

2. Type the IP address on the ROS end into the IP address field on the App. Both the robot and AR device must be on the same wifi network.

3. Select from the available visaualization options to view. 

4. Display the Targert image without modifying the scale. Currently, the target image placed on top of the Astra Camera.









