# Sanitizer-Robot-Project
Control of an autonomous mobile robot that has to perform some tasks in the **Turtlebot3** simulation environment using **ROS Noetic**. Course project of *Autonomous and Mobile Robotics* course at University of Bologna. Team members: Lorenzo Balandi, Salvatore Bamundo (https://github.com/Salvatore1999).

This repository includes:
* ROS packages used and developed for the project (Python and C++). *turtlebot3_big_house*  and *turtlebot3_project_house* are gazebo environments
* 2 textfiles containing the commands to be runned on multiple terminals to launch simulation and nodes
* Jupyter Notebooks used to manipulate the map image and to test the *room detection algorithm* and the algorithms used to find new goals to complete the map sanification
* final presentation.

The project uses the Turtlebot3 packages for SLAM and Navigation (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). We also used **explore_lite** package (http://wiki.ros.org/explore_lite) with the parameter min_frontier_size set to 0.3 and **OpenCV**.

**Warning**: the robot performs all the required tasks but it's not optimized and there are some parts of the code that lack flexibility and portability. Changes that may be done in the future are:
* Optimize behaviour of the robot to spend less time to sanify the map (optimize path, consider already energized areas...)
* Add functions to make the code flexible and easily reusable (e.g. coordinate transformation between image reference frame and Rviz reference frame

In brief, the robot has to perform the following **tasks**:
* Create a map of an unknown environment
* Localize itself in the map
* Reach some golas contained in a textfile
* Sanitize all the map with a minimum amount of energy (pretending that the robot is equipped with UV lamps)

![image](https://user-images.githubusercontent.com/100198704/156595631-bd50f70e-ce67-42dc-bdab-9c69ef50c8e0.png)
![map](https://user-images.githubusercontent.com/100198704/156595405-5d07c474-18ff-4ed8-982a-0ab7939f336d.png)
![image](https://user-images.githubusercontent.com/100198704/156595678-188fd558-f8c4-47be-a0f2-6eab590ee7c2.png)
![image](https://user-images.githubusercontent.com/100198704/156595542-a9dcec8f-6491-4b4c-9b2b-75edd4671cb6.png)
