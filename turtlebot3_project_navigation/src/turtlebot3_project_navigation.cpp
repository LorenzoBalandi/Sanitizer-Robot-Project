// example_move_base_client: 
// wsn, October, 2016
// client of move_base

/*
LB, 22 Jan 2022
Navigation node for AMR project

*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
//#include <xform_utils/xform_utils.h>

//----------------------------FUNCTIONS------------------------------------------------------------

geometry_msgs::PoseStamped g_destination_pose; // goal position and orientation


// set desired goal pose
void set_des_pose() {
    //g_destination_pose.header.frame_id="/map"; // the correct name is map
    g_destination_pose.header.frame_id = "map";  // we are working in the map reference frame
    g_destination_pose.header.stamp = ros::Time::now();
    g_destination_pose.pose.position.z = 0.0; // 2d navigation
    g_destination_pose.pose.position.x = 0.0;
    g_destination_pose.pose.position.y = 0.0;
    //g_destination_pose.pose.orientation.z= -0.707; 
    g_destination_pose.pose.orientation.w= 1.0;
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}


//----------------------------MAIN------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_project_navigation"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    

    move_base_msgs::MoveBaseGoal move_base_goal;
    geometry_msgs::PoseStamped robot_pose;

    set_des_pose(); //define values for via points


    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigator_ac("move_base", true);
    
    // CONNECTION TO move_base SERVER
    // attempt to connect to the server:
    ROS_INFO("waiting for move_base server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to move_base action server"); // if here, then we connected to the server; 
    
    // SEND GOAL TO NAVIGATION STACK
    //geometry_msgs/PoseStamped target_pose
    move_base_goal.target_pose = g_destination_pose;         
    ROS_INFO("sending goal: ");
    navigator_ac.sendGoal(move_base_goal,&navigatorDoneCb); 
        
    bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));
    // bool finished_before_timeout = navigator_ac.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        

return 0;
}
