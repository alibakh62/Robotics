# Introduction
Here, we want to use the power of ROS and implement an EKF package. 

In this lab, we've controlled a robot inside a simulator and you collect sensitive information from the robots inertial measurement unit, rotary encoders and vision sensors. 

Then, you'll apply an EKF ROS package to this robot. The EKF ROS package will compare data generated from the robot's on-boars sensors and apply sensor fusion to estimate the robot's pose as it moves around. 

Within this lab, you'll be working with five different ROS packages, each with a different purpose.

- **turtlebot_gazebo:** launches a mobile robot inside a gazebo environment. 
- **robot_pose_ekf:** estimates the position and orientation of the robot.
- **odom_to_trajectory:** opens the odometry values generated over time into a trajectory path.
- **turtlebot_teleop:** lets you drive the robot using keyboard commands.
- **rviz:** lets you visualize the estimated position and orientation of the robot.


