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
