# Overview
Here, we'll write nodes in C++. The first node that we'll write is called `simple_mover`. The `simple_mover` node does nothing more than publish joint angle commands to `simple_arm`.

Then, we'll write another node called `arm_mover`. The `arm_mover` node provides a service called `safe_move`, which allows the arm to be moved to any position within its workspace that has been deemed safe. The safe zone is bounded by minimum and maximum joint angles, and is configurable via the ROS parameter server.

The last node we'll write is the `look_away` node. This node subscribes to the arm joint positions and a topic where camera data is being published. When the camera detects an image with uniform color, meaning that it’s looking at the sky, and the arm is not moving, the node will call the `safe_move` service via a client to move the arm to a new position.

# ROS Publishers
Before we write the code for `simple_mover`, it may be helpful to see how ROS Publishers work in C++.

Publishers allow a node to send messages to a topic, so that data from the node can be used in other parts of ROS. In C++, ROS publishers typically have the following definition format, although other parameters and arguments are possible:

```cpp
ros::Publisher pub1 = n.advertise<message_type>("/topic_name", queue_size);
```

The `pub1` object is a publisher object instantiated from the `ros::Publisher` class. This object allows you to publish messages by calling the `publish()` function.

To communicate with ROS master in C++, you need a **NodeHandle**. The node handle `n` will fully initialize the node.

The `advertise()` function is used to communicate with ROS and inform that you want to publish a message on a given topic name. The `"/topic_name"` indicates which topic the publisher will be publishing to.

The `message_type` is the type of message being published on "/topic_name". For example, the string message data type in ROS is `std_msgs::String`.

The `queue_size` indicates the number of messages that can be stored in a queue. A publisher can store messages in a queue until the messages can be sent. If the number of messages stored exceeds the size of the queue, the oldest messages are dropped.

Once the publisher object `pub1` has been created, as above, a `message` with the specified data type can be published as follows:

```cpp
pub1.publish(msg);
```

For more information about C++ ROS publishers, see the documentation [here](http://docs.ros.org/jade/api/roscpp/html/classros_1_1Publisher.html).

**Quiz!** Assume that a queued message is typically picked up in an average time of 1/10th of a second with a standard deviation of 1/20th of a second, and your publisher is publishing at a frequency of 10Hz. Of the options below, which would be the best setting for `queue_size`?

**Answer!** Choosing a good queue_size is somewhat subjective, but since messages are picked up at roughly the same rate that they are published, a `queue_size` of 2 provides a little room for messages to queue without being too large.

# Simple Mover
You will now go through the process of implementing your first ROS node in C++. This node is called `simple_mover`. As its name implies, this node only has one responsibility,and that is to command joint movements for `simple_arm`.

**Goal**

The goal of the `simple_mover` node is to command each joint in the simple arm and make it swing between -pi/2 to pi/2 over time. Here’s a demonstration of this node in action in [this video](https://youtu.be/Ki5LkE_xir4).

## Topics

To do so, it must publish joint angle command messages to the following topics:

| | |
|--------------|--------------------------------------------------------------|
| **Topic Name**   | /simple_arm/joint_1_position_controller/command              |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 1 to move counter-clockwise, units in radians |
| **Topic Name**   | /simple_arm/joint_2_position_controller/command              |
| **Message Type** | std_msgs/Float64                                             |
| **Description**  | Commands joint 2 to move counter-clockwise, units in radians |

**NOTE:** If you no longer have the `catkin_ws` or `simple_arm` package from the previous lesson, you need to re-create a new `catkin_ws` and clone the package inside your `/home/workspace/catkin_ws/src` with:

```bash
$ mkdir -p /home/workspace/catkin_ws/src/
$ cd /home/workspace/catkin_ws/src/
$ git clone -b first_interaction https://github.com/udacity/RoboND-simple_arm/ simple_arm
```

## Adding the source directory
In order to create a new node in C++, you must first create the `src` directory within the `simple_arm` package as it does not yet exist.

```bash
$ cd /home/workspace/catkin_ws/src/simple_arm/
$ mkdir src
```

## Creating a new script
Once the source directory has been created, C++ scripts can be added to the package. Now, create the `simple_mover` C++ script inside the source directory of the package.

```bash
$ cd /home/workspace/catkin_ws/src/simple_arm/src/
$ touch simple_mover.cpp
```

## The code
Below is the complete code for the `simple_mover` C++ node, with line-by-line comments embedded. You can copy and paste this code into the `simple_mover` script you created in `/home/workspace/catkin_ws/src/simple_arm/src/` directory like this:

First, open a new terminal. Then:

```bash
$ cd /home/workspace/catkin_ws/src/simple_arm/src/
$ gedit simple_mover.cpp
```

Below is the code for `simple_mover`:

```cpp
#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    // Initialize the arm_mover node
    ros::init(argc, argv, "arm_mover");

    // Create a handle to the arm_mover node
    ros::NodeHandle n;

    // Create a publisher that can publish a std_msgs::Float64 message on the /simple_arm/joint_1_position_controller/command topic
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    // Create a publisher that can publish a std_msgs::Float64 message on the /simple_arm/joint_2_position_controller/command topic
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Set loop frequency of 10Hz
    ros::Rate loop_rate(10);

    int start_time, elapsed;

    // Get ROS start time
    while (not start_time) {
        start_time = ros::Time::now().toSec();
    }

    while (ros::ok()) {
        // Get ROS elapsed time
        elapsed = ros::Time::now().toSec() - start_time;

        // Set the arm joint angles
        std_msgs::Float64 joint1_angle, joint2_angle;
        joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

        // Publish the arm joint angles
        joint1_pub.publish(joint1_angle);
        joint2_pub.publish(joint2_angle);

        // Sleep for the time remaining until 10 Hz is reached
        loop_rate.sleep();
    }

    return 0;
}
```

See the video [here](https://youtu.be/mj7lwGqouEA)

## The code: Explained

```cpp
#include "ros/ros.h"
```

`ros` is the official client library for ROS. It provides most of the fundamental functionality required for interfacing with ROS via C++. It has tools for creating Nodes and interfacing with Topics, Services, and Parameters.

```cpp
#include "std_msgs/Float64.h"
```

From the `std_msgs` package, the Float64 header file is imported. The [std_msgs](http://wiki.ros.org/std_msgs) package also contains the primitive message types in ROS. Later, you will be publish Float64 messages to the position command topics for each joint.

```cpp
ros::init(argc, argv, "arm_mover");
```

A ROS node is initialized with the `init()` function and registered with the ROS Master. Here `arm_mover` is the name of the node. Notice that the main function takes both `argc` and `argv` arguments and passes them to the `init()` function.

```cpp
ros::NodeHandle n;
```

A node handle object `n` is instantiated from the NodeHandle class. This node handle object will fully initialize the node and permits it to communicate with the ROS Master.

```cpp
ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);
```

Two publishers are declared, one for joint 1 commands, and one for joint 2 commands. The node handle will tell the ROS master that a Float64 message will be published on the joint topic. The node handle also sets the queue size to 10 in the second argument of the advertise function.

```cpp
ros::Rate loop_rate(10);
```

A frequency of 10HZ is set using the `loop_rate` object. Rates are used in ROS to limit the frequency at which certain loops cycle. Choosing a rate that is too high may result in unnecessary CPU usage, while choosing a value too low could result in high latency. Choosing sensible values for all of the nodes in a ROS system is a bit of a fine art.

```cpp
start_time = ros::Time::now().toSec();
```

We set `start_time` to the current time. In a moment we will use this to determine how much time has elapsed. When using ROS with simulated time (as we are doing here), `ros-Time-now` will initially return 0, until the first message has been received on the `/clock` topic. This is why `start_time` is set and polled continuously until a nonzero value is returned.

```cpp
elapsed = ros::Time::now().toSec() - start_time;
```

In the main loop, the elapsed time is evaluated by measuring the current time and subtracting the start time.

```cpp
std_msgs::Float64 joint1_angle, joint2_angle;
joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
```

The joint angles are sampled from a sine wave with a period of 10 seconds, and in magnitude from [-pi/2, +pi/2].

```cpp
joint1_pub.publish(joint1_angle);
joint2_pub.publish(joint2_angle);
```

Each trip through the body of the loop will result in two joint command messages being published.

```cpp
loop_rate.sleep();
```

Due to the call to `loop_rate.sleep()`, the loop is traversed at approximately 10 Hertz. When the node receives the signal to shut down (either from the ROS Master, or via a signal from a console window), the loop will exit.

