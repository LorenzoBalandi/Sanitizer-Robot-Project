/*
 LB, 27 Jan 2022
 ROS node for the localizatio of the robot in AMR project
 
 Explanation:
 the robot turns around one time, then go forward and backward and asks to the user if the localization was successful,
 if not: clear costmap, redistribute particles, repeat movements
 if yes: localization finished.

USEFUL LINKS:
https://answers.ros.org/question/40758/calling-global-localization-service-in-c/
https://answers.ros.org/question/358746/how-to-use-rosservicecall/                  <--- BETTER WAY OF CALLING SERVICES

*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <iostream> // necessary to use cin function
#include <std_srvs/Empty.h> 

#define TWO_PI 6.28


geometry_msgs::Twist command; // global variable


//----------------------------FUNCTIONS------------------------------------------------------------

void stop() {
    // initialize the twist command to 0
    // used also to stop the robot while moving
    command.linear.x = 0.0;
    command.linear.y = 0.0;
    command.linear.z = 0.0;
    command.angular.x = 0.0;
    command.angular.y = 0.0;
    command.angular.z = 0.0;
    ROS_INFO("Stop");
}

//----------------------------MAIN------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_project_localization"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    ros::Publisher robot_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // this node will publish on cmd_vel topic
    ROS_INFO("INITIALIZING LOCALIZATION NODE");
    std_srvs::Empty srv;
    std_srvs::Empty srv2;
    /*
    Twist:
    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
    */

   char localization_result = 'n';

   stop(); // initialize command message
   robot_vel_pub.publish(command);

    do {
        // call service to redistribute particles
        if (ros::service::call("/global_localization",srv)) {
            ROS_INFO("AMCL particles redistributed!");
        }
        // cal service to clear costmaps
        if (ros::service::call("/move_base/clear_costmaps", srv2)) {
            ROS_INFO("Costmaps cleared!");
        }

        ros::Duration(1.0).sleep();
        
        command.angular.z = -0.3;
        ROS_INFO("Turning around...");
        robot_vel_pub.publish(command); // publish the rotation command
        ros::Duration(-TWO_PI/command.angular.z).sleep(); // wait until a complete round is done (the - is required to have a positive time)
        stop();
        robot_vel_pub.publish(command);
        command.linear.x = 0.1;
        ROS_INFO("Moving forward...");
        robot_vel_pub.publish(command); // publish linear velocity command
        ros::Duration(8.0).sleep();
        stop();
        robot_vel_pub.publish(command);
        command.linear.x = -0.1;
        ROS_INFO("Moving backward...");
        robot_vel_pub.publish(command); // publish linear velocity command
        ros::Duration(8.0).sleep();
        stop();
        robot_vel_pub.publish(command);

        ROS_INFO("Is the robot successfully localized? (y/n)");
        std::cin >> localization_result;

    } while(localization_result != 'y');



    ROS_INFO("LOCALIZATION SUCCESSFUL");
    // ADD POSITION IN THE MAP

    //ros::spin();
}

/*
// SEND GOAL TO NAVIGATION STACK
    char result;

    while (choice!='q') {


        ROS_INFO("Type any key to send a destination goal, q to quit");
        std::cin >> choice;
        if (choice=='q') {
            break; }
        set_des_pose(); //ask for values for via points
        //geometry_msgs/PoseStamped target_pose
        move_base_goal.target_pose = g_destination_pose;         
        ROS_INFO("sending goal: ");
        navigator_ac.sendGoal(move_base_goal,&navigatorDoneCb); 
        
        bool finished_before_timeout = navigator_ac.waitForResult(); // wait forever...

    }
    
    ROS_INFO("Quitting program");

    */