// example_move_base_client: 
// wsn, October, 2016
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
//#include <xform_utils/xform_utils.h>

//----------------------------FUNCTIONS------------------------------------------------------------

geometry_msgs::PoseStamped g_destination_pose;

class XformUtils {
public:
   geometry_msgs::PoseStamped get_pose_from_stamped_tf(tf::StampedTransform sTf);
   void printStampedPose(geometry_msgs::PoseStamped stPose); 
};


void set_des_pose() {
    //g_destination_pose.header.frame_id="/map"; // the correct name is map
    g_destination_pose.header.frame_id = "map";
    g_destination_pose.header.stamp = ros::Time::now();
    g_destination_pose.pose.position.z = 0;
    g_destination_pose.pose.position.x = -0.0;
    g_destination_pose.pose.position.y = 0.0;
    //g_destination_pose.pose.orientation.z= -0.707; 
    g_destination_pose.pose.orientation.w= 1.0;
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}

void XformUtils::printStampedPose(geometry_msgs::PoseStamped stPose) {
    ROS_INFO_STREAM("frame id = " << stPose.header.frame_id);
    ROS_INFO_STREAM("origin: " << stPose.pose.position.x << ", " << stPose.pose.position.y << ", " << stPose.pose.position.z);
    ROS_INFO_STREAM("quaternion: " << stPose.pose.orientation.x << ", " << stPose.pose.orientation.y << ", "
            << stPose.pose.orientation.z << ", " << stPose.pose.orientation.w);
}

geometry_msgs::PoseStamped XformUtils::get_pose_from_stamped_tf(tf::StampedTransform tf) {
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

//----------------------------MAIN------------------------------------------------------------

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    set_des_pose(); //define values for via points
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_pose;
    move_base_msgs::MoveBaseGoal move_base_goal;
    XformUtils xform_utils; //instantiate an object of XformUtils
    
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
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);

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
    xform_utils.printStampedPose(g_destination_pose);
    navigator_ac.sendGoal(move_base_goal,&navigatorDoneCb); 
        
    bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
        
    return 0;
}

