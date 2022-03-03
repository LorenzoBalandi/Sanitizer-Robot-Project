# Sanitizer-Robot-Project
Control of an autonomous mobile robot that has to perform some tasks in the **Turtlebot3** simulation environment using **ROS Noetic**. Course project of Autonomous and Mobile Robotics course at University of Bologna.

Team members: Lorenzo Balandi, Salvatore Bamundo (https://github.com/Salvatore1999).

**Warning**: the robot performs all the required tasks but it's not optimized and there are some parts of the code that lack flexibility and portability. Changes that may be done are:
* Optimize behaviour of the robot to spend less time to sanify the map (optimize path, consider already energized areas...)
* Add functions to make the code flexible and easily reusable (e.g. coordinate transformation between image reference frame and Rviz reference frame
