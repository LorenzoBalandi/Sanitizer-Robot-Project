#!/usr/bin/env python
'''
AMR project navigation: the robot reades goals from text files and reaches them
Balandi, Bamundo 27 Jan 2022
'''
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot3_project_navigation_textfile')

        with open("/home/lorenzo/turtle_ws/src/turtlebot3_project_navigation/scripts/goals.txt", "r") as file:
            lines = file.readlines()
            for line in lines:
                goal = line.split()
                goal_x = float(goal[0])
                goal_y = float(goal[1])
                rospy.loginfo("goal_x = %f", goal_x)
                rospy.loginfo("goal_y = %f", goal_y)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = goal_x
                goal.target_pose.pose.position.y = goal_y
                goal.target_pose.pose.orientation.w = 1.0
                result = movebase_client()
                if result:
                    rospy.loginfo("Goal execution done!")

  
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
