#!/usr/bin/env python

'''
28 Jan 2022
Balandi, Bamundo
AMR Project
'''

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from turtlebot3_project_navigation.msg import Array
import cv2
import numpy as np


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


# new function used to send the new_goals
def movebase_client_new_goals(goal_):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    client.send_goal(goal_)
    finished_before_timeout = client.wait_for_result(rospy.Duration(180.0)) 
    # if this goal takes more than 180 seconds go to the following goal
    # this is useful to avoid the robot to be stuck near an obstacle
    if not finished_before_timeout:
        rospy.logwarn("Giving up reaching this goal")
    else:
        return client.get_result()



def process_room(room):
    global img_gray
    # goals is a list of pixel coordinates in openCV frame
    # goals = [x1,y1,x2,y2,...]
    goals = []
    goal_1_x = room.x+round(room.height/4) + 6
    goal_1_y = room.y+round(room.width/4) + 6
    if img_gray[goal_1_x,goal_1_y] != 0: # check if the pixel is black
        goals.append(goal_1_x)
        goals.append(goal_1_y)
    else:
        goals.append(-1) # append -1 if pixel is black
        goals.append(-1)
    goal_2_x = room.x+round(room.height*(3/4)) + 6
    goal_2_y = room.y+round(room.width/4) + 6
    if img_gray[goal_2_x,goal_2_y] != 0: # check if the pixel is black
        goals.append(goal_2_x)
        goals.append(goal_2_y)
    else:
        goals.append(-1) # append -1 if pixel is black
        goals.append(-1)
    goal_3_x = room.x+round(room.height*(3/4)) + 6
    goal_3_y = room.y+round(room.width*(3/4)) + 6
    if img_gray[goal_3_x,goal_3_y] != 0: # check if the pixel is black
        goals.append(goal_3_x)
        goals.append(goal_3_y)
    else:
        goals.append(-1) # append -1 if pixel is black
        goals.append(-1)
    goal_4_x = room.x+round(room.height/4) + 6
    goal_4_y = room.y+round(room.width*(3/4)) + 6
    if img_gray[goal_4_x,goal_4_y] != 0: # check if the pixel is black
        goals.append(goal_4_x)
        goals.append(goal_4_y)
    else:
        goals.append(-1) # append -1 if pixel is black
        goals.append(-1)
    
    return(goals)


# enter here when energy_cv send the pixels not energized
# this function computes the goals yo send to move base in order to comlete the energization of the map
def new_goals_callback(data):
    resolution = 0.2
    w = 80
    h = 80
    global new_goals # new goals to send to move base

    print("In new_goals callback")
    new_goals = data.array
    print(len(new_goals))
    print(type(new_goals))
    print(new_goals)

    goal_ = MoveBaseGoal() # initialize goal
    goal_.target_pose.header.frame_id = "map" # same for all goals
    goal_.target_pose.pose.orientation.w = 1.0 # same for all goals
    for i in range(0, len(new_goals),2):
        x_px_goal = new_goals[i]
        y_px_goal = new_goals[i+1]
        # transformation from pixel coordinates in map coordinates [m]
        # Big house:
        x_pos_goal = -resolution*(x_px_goal-h/2+3)
        y_pos_goal = -resolution*(y_px_goal-w/2+2)
        # Project house:
        
        print(x_pos_goal)
        print(y_pos_goal)
        goal_.target_pose.header.stamp = rospy.Time.now()
        goal_.target_pose.pose.position.x = x_pos_goal
        goal_.target_pose.pose.position.y = y_pos_goal
        # send goal
        result_=False
        rospy.loginfo("Sending goal_: %f, %f", x_pos_goal,y_pos_goal)
        result_ = movebase_client_new_goals(goal_)
        if result_:
            rospy.loginfo("Goal execution done!")
    rospy.loginfo("Map sanified!")




class room:
    def __init__(self,x,y,width,height): # self is needed!
        self.x = x
        self.y = y
        self.width = width
        self.height = height




################# MAIN ######################
if __name__ == '__main__':
    global img_gray
    global new_goals
    new_goals = []

    rospy.init_node('navigation_energy', anonymous=True)
    publisher = rospy.Publisher("goals_finished", Bool, queue_size=10)
    rospy.Subscriber("new_goals", Array, new_goals_callback)

    rospy.loginfo("ROOMS DETECTION AND NAVIGATION NODE")

    goals_finished = False # boolean used to tell the energy_cv node to check if all the rooms are sanified
    
    # necessary when we need to publish a single message on a topic
    crtl_c = False
    while not crtl_c:
        connections = publisher.get_num_connections()
        if connections > 0:
            goals_finished = False # boolean used to tell the energy_cv node to check if all the rooms are sanified
            publisher.publish(goals_finished)
            print("Published!")
            crtl_c = True
        else:
            rospy.Rate(1).sleep()

    # Uncomment in case of project_house map
    #rospy.wait_for_service('/move_base/clear_costmaps')
    #clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    #print ("Costmap cleared!")


    image = cv2.imread('/home/lorenzo/map_black.png')
    rospy.loginfo("Map imported successfully!")
    # convert the image to grayscale format
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    print(img_gray.shape)

    # perform erosion and dilation in order to "separate" the rooms
    kernel = np.ones((3,3),np.uint8) # big house map
    #kernel = np.ones((5,5),np.uint8) # project house map
    erosion = cv2.erode(img_gray,kernel,iterations = 10)
    dilate = cv2.dilate(erosion,kernel,iterations = 10)

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, hierarchy = cv2.findContours(image=dilate, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # convert to rgb (just for visualization)
    image_rgb = cv2.cvtColor(dilate,cv2.COLOR_BGR2RGB)

    n_rooms = 0

    # list to store the coordinates of the top left corner of the rooms and their width and height
    rooms = []

    for contour in contours: 
        x,y,w,h = cv2.boundingRect(contour) # x,y top-left coordinate of the rectangle and w,h be its width and height
        area = w*h
        if area > 150: # discard tables legs
            n_rooms+=1 
            r = room(y,x,w,h) # width is y and height is x in openCV frame!  
            rooms.append(r)  
            #cv2.rectangle(image_rgb,(x,y),(x+w,y+h),(0,0,255),2)

    # now rooms is a list of room of size n_rooms

    rospy.loginfo("In this house I found %d rooms.", n_rooms)

    # Show the updated image 
    #cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
    #cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
    #cv2.imshow('image_rsz',image_rgb) # display small image in big window
    #cv2.waitKey(0)

    # data used for coordinate transformation
    x_pixel = img_gray.shape[0] # number of pixels (384 if map_black.png)
    y_pixel = img_gray.shape[1]
    resolution = 0.05 # meters/pixel
    
    goal = MoveBaseGoal() # initialize goal
    goal.target_pose.header.frame_id = "map" # same for all goals
    goal.target_pose.pose.orientation.w = 1.0 # same for all goals
    room_index = 1
    '''
    meno_rooms = []
    for i in range(0,1):
        meno_rooms.append(rooms[i])
    print("La meno_rooms ha ",len(meno_rooms),"stanze")
    '''
    for room_ in rooms:
        rospy.loginfo("Processing room number %d...", room_index) # 
        room_index+=1
        goals = process_room(room_)
        print("Goals:",goals)
        i = 0
        n_points = 4 # number of points to reach in each room
        # cannot use for goal_ in goals because we have to access index n and n+1
        for n in range(n_points):
            check_number = goals[i]
            if check_number != -1: # check if pixel is black, in that case it is skipped
                # at this point goals[i] is the x position in pixel, goals[i+1] is the y position
                # transform goals in map reference frame
                print(goals[i])
                print(goals[i+1])
                goal_x_map = -resolution*(goals[i]-x_pixel/2) #+ 8*resolution # +8 is empirical to compensate the misalignment between the frames
                goal_y_map = -resolution*(goals[i+1]-y_pixel/2) #+ 12*resolution
                # Uncomment in case of project_house map
                #goal_x_map = resolution*(goals[i+1]-y_pixel/2)
                #goal_y_map = -resolution*(goals[i]-x_pixel/2)
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = goal_x_map
                goal.target_pose.pose.position.y = goal_y_map
                # send goal
                rospy.loginfo("Sending goal: %f, %f", goal_x_map,goal_y_map)
                result = movebase_client()
                if result:
                    rospy.loginfo("Goal execution done!")
            else:
                rospy.logwarn("Goal skipped because coincident with an obstacle")
            i+=2 # go to next point
    
    rospy.loginfo("Reached all goals!")
    rospy.loginfo("Checking if sanification complete...")

    # necessary when we need to publish a single message on a topic
    crtl_c = False
    while not crtl_c:
        connections = publisher.get_num_connections()
        if connections > 0:
            goals_finished = True # boolean used to tell the energy_cv node to check if all the rooms are sanified
            publisher.publish(goals_finished)
            print("Published!")
            crtl_c = True
        else:
            rospy.Rate(1).sleep()

    # receive new goals
    print(len(new_goals))
    print(type(new_goals))
    print(new_goals)

    # TO DO
    # check process_room function
    
    # check if all the map has been sanitized
    # per definire un tipo di messaggio vedere i link:  
    # https://answers.ros.org/question/9471/how-to-recieve-an-array-over-publisher-and-subscriber-python/
    # http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Using_msg
    
    rospy.spin()