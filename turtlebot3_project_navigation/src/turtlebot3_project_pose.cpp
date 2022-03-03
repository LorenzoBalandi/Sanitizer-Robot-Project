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

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}

void printStampedPose (geometry_msgs::PoseStamped stPose) {
    ROS_INFO_STREAM("   frame id = " << stPose.header.frame_id);
    ROS_INFO_STREAM("   origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z);
    ROS_INFO_STREAM("   quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w);
}

geometry_msgs::PoseStamped get_pose_from_stamped_tf(tf::StampedTransform tf) {
    //clumsy conversions--points, vectors and quaternions are different data types in tf vs geometry_msgs
    geometry_msgs::PoseStamped stPose;
    geometry_msgs::Quaternion quat; //geometry_msgs object for quaternion
    tf::Quaternion tfQuat; // tf library object for quaternion
    tfQuat = tf.getRotation(); // member fnc to extract the quaternion from a transform
    quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
    quat.y = tfQuat.y();
    quat.z = tfQuat.z();
    quat.w = tfQuat.w();
    stPose.pose.orientation = quat; //set the orientation of our PoseStamped object from result

    // now do the same for the origin--equivalently, vector from parent to child frame 
    tf::Vector3 tfVec; //tf-library type
    geometry_msgs::Point pt; //equivalent geometry_msgs type
    tfVec = tf.getOrigin(); // extract the vector from parent to child from transform
    pt.x = tfVec.getX(); //copy the components into geometry_msgs type
    pt.y = tfVec.getY();
    pt.z = tfVec.getZ();
    stPose.pose.position = pt; //and use this compatible type to set the position of the PoseStamped
    stPose.header.frame_id = tf.frame_id_; //the pose is expressed w/rt this reference frame
    stPose.header.stamp = tf.stamp_; // preserve the time stamp of the original transform
    return stPose;
}

geometry_msgs::PoseStamped localize_robot() {
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_pose;

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
    current_pose = get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    printStampedPose(current_pose);
    return current_pose;
}

//----------------------------MAIN------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_project_pose"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    ros::Publisher robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
    ros::Rate loop_rate(1); // Hz

    geometry_msgs::PoseStamped robot_pose;

    double x_pose; // x coordinate of the robot in the map reference frame
    double y_pose; // y coordinate of the robot in the map reference frame

    while (ros::ok()) {

    robot_pose = localize_robot();
    x_pose = robot_pose.pose.position.x;
    y_pose = robot_pose.pose.position.y;

    ROS_INFO_STREAM("x_pose of the robot = " << x_pose << ", y_pose of the robot = " << y_pose);

    robot_pose_pub.publish(robot_pose);
    ROS_INFO("Publishing robot_pose on 'robot_pose' topic");

    ros::spinOnce();
    loop_rate.sleep();

    }

return 0;
}
