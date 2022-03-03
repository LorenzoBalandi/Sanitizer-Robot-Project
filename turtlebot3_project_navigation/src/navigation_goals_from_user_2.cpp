// example_move_base_client that accepts inputs from the terminal: 
// wsn, October, 2016
// LB, 10 dec 21
// client of move_base

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <iostream> // necessary to use cin function

//----------------------------FUNCTIONS------------------------------------------------------------

geometry_msgs::PoseStamped g_destination_pose;


void set_des_pose() {
    double x_goal, y_goal;
    ROS_INFO("Insert x coordinate of goal (double): ");
    std::cin >> x_goal;
    ROS_INFO("Insert y coordinate of goal (double): ");
    std::cin >> y_goal;
    g_destination_pose.header.frame_id = "map";
    g_destination_pose.header.stamp = ros::Time::now();
    g_destination_pose.pose.position.x = x_goal;
    g_destination_pose.pose.position.y = y_goal;
    g_destination_pose.pose.position.z = 0;
    g_destination_pose.pose.orientation.w= 1.0;
    ROS_INFO("g_destination_pose succesfully set!");
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}


//----------------------------MAIN------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_pose;
    move_base_msgs::MoveBaseGoal move_base_goal;
    
    // COORDINATE TRANSFORMATION BTW base_link AND map FRAME
    bool tferr=true;
    ROS_INFO("waiting for tf between map and base_link...");
    tf::StampedTransform tfBaseLinkWrtMap; 
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
                tfListener.lookupTransform("map","base_link", ros::Time(0), tfBaseLinkWrtMap);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good; current pose is:");
    

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
    char choice = 'g';

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

    return 0;
}

