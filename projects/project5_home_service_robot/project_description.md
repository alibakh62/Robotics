# Overview
The goal of this project is to program a robot that can autonomosly map an environment and navigate to pick up and drop virtual objects.This project will mainly test your abilities and skills in setting the correct parameters, communicating between different nodes of different packages, coding nodes with C++, publishing and subscribing to different topics, writing launch files, and many others.

The home service robot project is composed of multiple steps:

- First, you will learn the "Building Editor" in Gazebo, which is a user-friendly tool that enables you to professionally model real-world environments in simulation. 
- Then, you will interface SLAM with with a "Wall Follower" node to autonomosly map an environment. 
- After building the map, you'll test your robot's ability in reaching multiple goals and orient itself with respect to them using the ROS "Navigation" stack.
- Finally, you will model virtual objects with "Markers" and learn how to deploy them with ROS. 

[video](https://youtu.be/WchXpePVMuI).

# Working Environment

In this project, Udacity provides you a in Classroom Workspace with ROS set up for you in the  [Project Workspace concept](https://classroom.udacity.com/nanodegrees/nd209/parts/75c8f42b-c844-4f61-b3c6-521956c5cf70/modules/de8554d1-78db-4d1d-9a78-9b9a93d2879e/lessons/1ccf2893-a07b-41c5-b2ed-7cdc48bd26fc/concepts/acd9e789-8460-48a7-ace8-9f6b2aea25c9). If you are not familiar with the Workspace, please review the  [Workspace Introduction lessons](https://classroom.udacity.com/nanodegrees/nd209/parts/0778207d-f34a-4178-8ccf-9e06b5bd2203/modules/48156d08-abb1-4c03-a18d-9db738a0b92b/lessons/e0c61e8d-7eac-4807-8737-d2bd321ae7a2/concepts/47784838-aea6-4834-9ebb-79fbb3e135af?contentVersion=2.0.0&contentLocale=en-us)!

<p align="center">
<img src="img/udacity-workspace.png" alt="drawing" width="700"/>
</p>

After you enter the Workspace Desktop, please upgrade the system using the command

```bash
sudo apt-get update && apt-get upgrade
```

# Shell Scripts

A shell script is a file containing a series of commands and could be executed. It is commonly used to set up environment, run a program, etc.

You already know how to build a  `roslaunch`  file. It is very convenient to launch multiple ROS nodes and set parameters from a single  `roslaunch`  command. However, when developing robotic software with different packages, it might get harder to track errors and bugs generated from different nodes.

That's when shell scripts come in handy! After you create a shell script file to launch one or many nodes each in separate terminals, you will have the power to track the output of different nodes and keep the convenience of running a single command to launch all nodes.

## Your  `launch.sh`  Script

Let us start by creating this  `launch.sh`  script in the Udacity Workspace. Its goal is to launch Gazebo and Rviz in separate instances of terminals. Note that we are using  `xterm`  terminal in the script here.

```shell
#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " rosrun rviz rviz" 
```

The  `launch.sh`  shell script launches three terminals and issues one or multiple commands in each terminal. Let’s break down this script to understand the meaning of each line.

### Code Breakdown

`#!/bin/sh`

This statement is called a  `shebang`. It must be included in every shell script you write since it specifies the full path of the UNIX interpreter to execute it.

`xterm -e " gazebo " &`

With the  `xterm -e`  statement, we launch a new instance of an  `xterminal`. Inside this terminal, we launch gazebo using the command  `"gazebo"`. Then we add an ampersand  `&`  to indicate that another instance of an xterm terminal will be created in a separate statement.

`sleep 5`

We pause this script for 5 seconds using  `sleep`.

`xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &`

We launch a second instance of the xterm terminal. Inside this terminal, we source the ROS workspace and launch the ROS master node.

`sleep 5`

We pause this script for another 5 seconds.

`xterm -e " rosrun rviz rviz"`

We are launching a third instance of the xterm terminal, and running rviz.

Save your script file and give it  `execute`  pemission by  `chmod +x launch.sh`. Then launch the shell script with  `./launch.sh`.

After launching this script, we’ll have three open xterm terminals, and we will be able to track any errors or bugs that occur. To recap, this script will open the first terminal and launch gazebo. Then it will pause for 5 seconds and open a second terminal to launch the ROS master. It will pause for another 5 seconds and, finally, open a third terminal to launch RVIZ.

Try to launch your script in the Workspace and verify its functions!

# Simulation Setup

### Catkin Workspace

To program your home service robot, you will need to interface it with different ROS packages. Some of these packages are  **official ROS packages**  which offer great tools and others are  **packages that you’ll create**. The goal of this section is to prepare and build your  `catkin workspace`.

Here’s the list of the official ROS packages that you will need to grab, and other packages and directories that you’ll need to create at a later stage as you go through the project. Your  `catkin_ws/src`  directory should look as follows:

### Official ROS packages

Import these packages now and install them in the  `src`  directory of your  `catkin workspace`. Be sure to clone the full GitHub directory and not just the package itself.

1.  [gmapping:](http://wiki.ros.org/gmapping)  With the  **gmapping_demo.launch**  file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
2.  [turtlebot_teleop:](http://wiki.ros.org/turtlebot_teleop)  With the  **keyboard_teleop.launch**  file, you can manually control a robot using keyboard commands.
3.  [turtlebot_rviz_launchers:](http://wiki.ros.org/turtlebot_rviz_launchers)  With the  **view_navigation.launch**  file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
4.  [turtlebot_gazebo:](http://wiki.ros.org/turtlebot_gazebo)  With the  **turtlebot_world.launch**  you can deploy a turtlebot in a gazebo environment by linking the world file to it.

#### Your Packages and Directories

You’ll install these packages and create the directories as you go through the project.

1.  **map:**  Inside this directory, you will store your gazebo world file and the map generated from SLAM.
2.  **scripts:**  Inside this directory, you’ll store your shell scripts.
3.  **rvizConfig:**  Inside this directory, you’ll store your customized rviz configuration files.
4.  **pick_objects:**  You will write a node that commands your robot to drive to the pickup and drop off zones.
5.  **add_markers:**  You will write a node that model the object with a marker in rviz.

Your package should look like this now:

```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file                   
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file      
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file 
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├── map                          # map files
    │   ├── ...
    ├── scripts                   # shell scripts files
    │   ├── ...
    ├──rvizConfig                      # rviz configuration files
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```

# SLAM Testing

The next task of this project is to autonomously map the environment you designed earlier with the Building Editor in Gazebo. But before you tackle autonomous mapping, it’s important to test if you are able to manually perform SLAM by teleoperating your robot. The goal of this step is to manually test SLAM.

Write a shell script  `test_slam.sh`  that will deploy a turtlebot inside your environment, control it with keyboard commands, interface it with a SLAM package, and visualize the map in  `rviz`. We will be using turtlebot for this project but feel free to use your personalized robot to make your project stand out!

## Run and Test

Launch your test_slam.sh file, search for the  `xterminal`  running the  `keyboard_teleop`node, and start controlling your robot. You are not required to fully map your environment but just make sure everything is working fine. You might notice that the map is low quality, but don’t worry about that for now. If everything seems to be working fine, move on to the next concept!

<p align="center">
<img src="img/l6-c6-testing-slam.png" alt="drawing" width="700"/>
</p>

# Localization and Navigation Testing

The next task of this project is to pick two different goals and test your robot's ability to reach them and orient itself with respect to them. We will refer to these goals as the pickup and drop off zones. This section is only for testing purposes to make sure our robot is able to reach these positions before autonomously commanding it to travel towards them.

We will be using the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from start to goal position. The ROS navigation stack permits your robot to avoid any obstacle on its path by re-planning a new trajectory once your robot encounters them. You are familiar with this navigation stack from the localization project where you interfaced with it and sent a specific goal for your robot to reach while localizing itself with AMCL. If you are planning to modify the ROS navigation algorithm or you are curious to know how it's done, take a look at this official tutorial which teaches you how to write a global path planner as a plugin in ROS.

See [this video](https://youtu.be/t-0li7H131A) to learn how to manually command the robot.

## Test it

Once you launch all the nodes, you will initially see the particles around your robot, which means that AMCL recognizes the initial robot pose. Now, manually point out to two different goals, one at a time, and direct your robot to reach them and orient itself with respect to them.

# Reaching Multiple Goals

Earlier, you tested your robot capabilities in reaching multiple goals by manually commanding it to travel with the 2D NAV Goal arrow in rviz. Now, you will write a node that will communicate with the ROS navigation stack and autonomously send successive goals for your robot to reach. As mentioned earlier, the ROS navigation stack creates a path for your robot based on  **Dijkstra's**  algorithm, a variant of the  **Uniform Cost Search**  algorithm, while avoiding obstacles on its path.

There is an official ROS tutorial that teaches you how to send a single goal position and orientation to the navigation stack. You are already familiar with this code from the Localization project where you used it to send your robot to a pre-defined goal. Check out the  [tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals)  and go through its documentation.

Here’s the C++ code of this node which sends a  **single goal**  for the robot to reach. I included some extra comments to help you understand it:

```cpp
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
```

Watch the [video](https://youtu.be/RcbjN6KFJq8).

## Customize the code

You will need to modify this code and edit its node name to  **pick_objects**. Then, edit the  **frame_id**  to  `map`, since your fixed frame is the map and not base_link. After that, you will need to modify the code and include an extra goal position and orientation for your robot to reach.

The first goal should be your desired pickup goal and the second goal should be your desired drop off goal. The robot has to travel to the desired pickup zone, display a message that it reached its destination, wait 5 seconds, travel to the desired drop off zone, and display a message that it reached the drop off zone.

# Modeling Virtual Objects

The final task of this project is to model a virtual object with markers in rviz. The virtual object is the one being picked and delivered by the robot, thus it should first appear in its pickup zone, and then in its drop off zone once the robot reaches it.

First, let’s see how markers can be drawn in rviz. Luckily, there’s an official ROS tutorial that teaches you how to do it. The tutorial is an excellent reference and includes a C++ node capable of drawing basic shapes like arrows, cubes, cylinders, and spheres in rviz. You will learn how to define a marker, scale it, define its position and orientation, and finally publish it to rviz. The node included in the tutorial will publish a different shape each second at the same position and orientation. Check out the  [tutorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)  and go through the documentation to get started.

You will need to first run this node and visualize the markers in rviz. Then you’ll need to modify the code and publish a single shape example: a cube. Your code should follow this  **algorithm**:

-   Publish the marker at the pickup zone
-   Pause 5 seconds
-   Hide the marker
-   Pause 5 seconds
-   Publish the marker at the drop off zone

Later you will be able to combine this node with the pick_objects node coded earlier to simulate the full home service robot.

Watch [the video](https://youtu.be/Gq_FTu48ytc).

# Your Home Service Robot

Now it’s time to simulate a full home service robot capable of navigating to pick up and deliver virtual objects. To do so, the  **add_markers**  and  **pick_objects**  node should be communicating. Or, more precisely, the  **add_markers**  node should subscribe to your  **odometry**  to keep track of your robot pose.

Modify the  **add_markers**  node as follows:

-   Initially show the marker at the pickup zone
-   Hide the marker once your robot reaches the pickup zone
-   Wait 5 seconds to simulate a pickup
-   Show the marker at the drop off zone once your robot reaches it

## Note

There are many ways to solve this problem. To establish communications between the robot and the markers, one method already mentioned is to let your  `add_markers`  node subscribe to your robot odometry and keep track of your robot pose.

Other solutions to this problem might be to use ROS  [parameters](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams), subscribe to the AMCL pose, or even to publish a new variable that indicates whether or not your robot is at the pickup or drop off zone. Feel free to solve this problem in any way you wish.

Watch [the video](https://youtu.be/MSiU6SB__Dk).

# Project Submission
Once you have completed your project, use the [**Project Rubric**](https://review.udacity.com/#!/rubrics/2396/view) to review the project. If you have covered all of the points in the rubric, then you are ready to submit! If you see room for improvement in **any** category in which you do not meet specifications, keep working!

