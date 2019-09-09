# Introduction to SLAM

This section is about how robots learn to map their envinronment and **SLAM (Simultaneous Localization And Mapping)**. In localization, we estimate the robot's pose given the map of the environment. So, given the map and robot's access to sensor and movement, it can pose. But, _what if the map of environment doesn't exist?_ This can happen either to a changing environment or maybe just the fact that map doesn't exist. In such a case, the robot will have to construct a map. This leads us to **robotic mapping**.

**Mapping:** In mapping, the robot produces a map of the environment given the its pose(s). Therefore, given the pose and access to sensor and movement data, robot creates a map of the surrounding. _**However**_, in real world, even knowing the pose can be uncommon. This is where **SLAM** comes in. 

**SLAM:** with access to only sensor and movement data, the bot must simultaneously estimate the pose and produce the map of the environment and localizing itself relative to the map.

To recap, in localization, assuming the map, bot estimates its pose whereas in mapping, assuming the pose, the bot estimates the environment. 

**Challenge in mapping:** Pose usually consists of few variables (depends on the robot) that describe the state. But, mapping happens in continuous space, so there is infinite possibilities. Combine this with the uncertainties present in perception and the mapping task becomes even more challenging. There are also some other challenges. For example, the geometry of some of the surrounding objects might look alike and repeated throughout the environment (e.g. car driving in a street where similar trees are planted). 

There are number of algorithms for mapping. Here, we focus on **Occupancy Grid Mapping** algorithm. It divides the environment into finite number of grid cells. By estimating the state of each individual cell, it'll come up with a map of environment. 

Next, we move to **SLAM**. In SLAM, the robot must create the map while simultaneously localizing itself relative to the map. This is more challenging than both localization and mapping since neither the map nor the pose are provided. The existing uncertainty in both map and pose, make the robot's estimate of both _**correlated**_. In another words, the accuracy of the map depends on the accuracy of localization and vice versa. SLAM is often called the "chicken or egg" problem. The map is needed for localization and the robot's pose needed for mapping. _**This challenge is fundamental to robotics**_. For robots to be useful, they must be able to move around the environment they've never seen before (e.g. robot vacuum cleaner or self-driving cars). 

**SLAM Algorithms**:

- Extended Kalman Filter SLAM (EKF)
- Sparse Extended Information Filter (SEIF)
- Extended Information Form (EIF)
- FastSLAM
- GraphSLAM

We'll be covering "FastSLAM" and "GraphSLAM" here.

# Occupancy Grid Mapping

**Localization:**

- Assumption: Known Map
- Estimation: Robot's Trajectory

**Mapping:**

- Assumption: Robot's Trajectory
- Estimation: Map

Here, we'll learn how to map an environment with the Occupancy Grid Mapping algorithm!

**The importance of mapping:** Mapping is important because environment is dynamic. Even with a given map, there's always a chance that something changes. So, the robot needs capability to simulatneously update the map.

## Mapping challenges: 
There are two main challenges with mapping:

- **Unknown Map and Poses:** Both map and poses are unknown to us, so we either have to assume the map and estimate the pose or assume the pose and estimate the map and localize robot with respect to it. Estimating the map is a challenging problem because of the large number of variables. This can be solved using the _**occupancy grid**_ algorithm. Estimating pose and map when both are unknown will be covered in SLAM. 
- **Hypothesis space is huge:** This is because the hypothesis is huge, so when robots are deployed into an open environment where the robot has to sense an infinite number of objects. The _occupancy grid mapping_ provides a discrete representation of the map. But even with that approximation, the space of all possible maps will be large. So, the challenge is to estimate the full posterior maps for maps with high dimensional spaces. The Bayes posterior approach used in localization will diverge. An extension to it need to be used to accomodate the huge hypothesis space. 

To map an environment, we need information about walls and objects. For example, we can deploy a robot with a laser range finder sensor. The robot collects sensory information to detect obstacle around it. By using one of the mapping algorithms, we can group this data into a resulting map. However, there are difficulties in using this data as follows:

- **Size:** mapping large spaces is difficult because there's large amonut of data needs to be processed. The robot has to collect the information from all the sensors, combine them all to form a map and localize the robot every moment. This becomes particularly challenging when the size of the map is larger than the robot's perceptual range. 
- **Noise:** There is always noise associated with sensory data, which needs to be filtered. 
- **Perceptual ambiguity:** The ambiguity occurs when two places look alike. So, the robot needs to know at which point it passed which one. This is particularly important when robot travels in cyclic manner. In cyclic travels, the odometry accumulate errors and at the end of the cycle the error is large. 

## Mapping with known poses
In mapping with known poses, poses (`X`) are known, we also have the measurements (`Z`). Then, using a mapping algorithm (say occupancy grid mapping), we can estimate the posterior map given the noisy measurements and known poses. **However**, in most robotic applications, the odometry dats is noisy, so the robot poses are unknown to us. _So, why mapping is necessary under such a situation?_ **Mapping usually happens after SLAM**. So, the power of mapping is its post-processing. In SLAM, the problem changes from mapping with known poses to mapping with unknown poses. During SLAM, the robot will build a map of the environment, localize itself with respect to the map. After SLAM, the **occupancy grid algorithm** uses the exact poses filtered from SLAM. Then, with the known poses from SLAM and noisy measurements, generates a map for path planning and navigation. 



