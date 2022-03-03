# Sanitizer-Robot-Project
Control of an autonomous mobile robot that has to perform some tasks in the **Turtlebot3** simulation environment using **ROS Noetic**. Course project of *Autonomous and Mobile Robotics* course at University of Bologna. Team members: Lorenzo Balandi, Salvatore Bamundo (https://github.com/Salvatore1999).

This repository includes the ROS packages used and developed for the project, 2 textfiles containing the commands to be runned on multiple terminals to launch simulation and nodes and the final presentation. The project uses the Turtlebot3 packages for SLAM and Navigation (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). We also used **explore_lite** package (http://wiki.ros.org/explore_lite) with the parameter min_frontier_size set to 0.3 .

**Warning**: the robot performs all the required tasks but it's not optimized and there are some parts of the code that lack flexibility and portability. Changes that may be done in the future are:
* Optimize behaviour of the robot to spend less time to sanify the map (optimize path, consider already energized areas...)
* Add functions to make the code flexible and easily reusable (e.g. coordinate transformation between image reference frame and Rviz reference frame

In brief, the robot has to perform the following **tasks**:
* Create a map of an unknown environment
* Localize itself in the map
* Reach some golas contained in a textfile
* Sanitize all the map with a minimum amount of energy (pretending that the robot is equipped with UV lamps)
