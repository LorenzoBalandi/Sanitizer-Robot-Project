#!/usr/bin/env python

'''
23 Jan 2022
Balandi, Bamundo
AMR Project
'''

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from turtlebot3_project_navigation.msg import Array
import cv2
import numpy as np


def goals_finished_callback(data):
    global goals_finished
    print("Goals_finished is",data.data)
    goals_finished = data.data

def energy_callback(data):

    global image_rgb
    global dt
    global energy_const
    global energy_matrix
    global dist_
    global goals_finished

    ## COMPUTE ENERGY
    if goals_finished is False:

        x_pos = data.pose.position.x
        y_pos = data.pose.position.y
        rospy.loginfo("I heard x_pos = %f", x_pos)
        rospy.loginfo("I heard y_pos = %f", y_pos)
        # Coordinate transformation from rviz [m] to image [m]
        x_pixel = 80 # number of pixels
        y_pixel = 80
        #x_pixel = 96 # number of pixels (project_house map)
        #y_pixel = 208
        resolution = 0.2 # meters/pixel
        # Coordinate transformation from image [m] to image [pixel]
        # Big house map:
        y_px = -y_pos/resolution + (y_pixel/2) - 2 # -2 is empirical to counterbalance the origin of the map frame not being precisely at the center
        x_px = -x_pos/resolution + (x_pixel/2) - 3
        # Project_house map:
        #y_px = x_pos/resolution + (y_pixel/2)
        #x_px = -y_pos/resolution + (x_pixel/2)
        
        x_px = round(x_px) # round to nearest integer
        y_px = round(y_px)
        if x_px < 0 or y_px < 0:
            # set a warning if pixel coordinates are < 0 (this should never happen)
            rospy.logwarn("WARNNING: negative pixel coordinates!")
        rospy.loginfo("x_pos in pixels = %d", x_px)
        rospy.loginfo("y_pos in pixels = %d", y_px)

        ## END PIXEL RECOGNITION

        ## UP LEFT
        image_rgb, energy_matrix = up_left(x_px, y_px,image_rgb, energy_matrix)

        ## UP RIGHT
        image_rgb, energy_matrix = up_right(x_px, y_px,image_rgb, energy_matrix)

        ## DOWN RIGHT
        image_rgb, energy_matrix = down_right(x_px, y_px,image_rgb, energy_matrix)

        ## DOWN RIGHT
        image_rgb, energy_matrix = down_left(x_px, y_px,image_rgb, energy_matrix)

        ## ONLY UP
        image_rgb, energy_matrix = only_up(x_px, y_px,image_rgb, energy_matrix)

        ## ONLY DOWN
        image_rgb, energy_matrix = only_down(x_px, y_px,image_rgb, energy_matrix)

        limit_up = x_px - dist_
        if limit_up < 0:
            limit_up = 0

        limit_left = y_px - dist_
        if limit_left < 0:
            limit_left = 0

        limit_right = y_px + dist_
        if limit_right >= energy_matrix.shape[1]-1:
            limit_right = energy_matrix.shape[1]

        limit_down = x_px + dist_
        if limit_down >= energy_matrix.shape[0]-1:
            limit_down = energy_matrix.shape[0]    
        
        for i in range(limit_up, limit_down):
            for j in range(limit_left , limit_right):
                energy_matrix[i,j,1] = 0

        # Show the updated image 
        cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
        cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
        cv2.imshow('image_rsz',image_rgb) # display small image in big window
        cv2.waitKey(5)

    ## All the goals are finished, check new goals
    else:
        #image_rgb_save = cv2.cvtColor(image_rgb,cv2.COLOR_BGR2RGB)
        image_rgb_save = np.copy(image_rgb)
        cv2.imwrite("/home/lorenzo/map_from_code.png", image_rgb_save)
        new_goals = check_new_goals(image_rgb)  #new_goals is a row list in which each element is a coordinate of the new goal
        # [x1 y1 x2 y2 ...]
        new_goals_msg =  Array()
        new_goals_msg.array = new_goals
        goals_finished = False
        # necessary when we need to publish a single message on a topic    
        crtl_c = False
        while not crtl_c:
            connections = new_goals_publisher.get_num_connections()
            if connections > 0:
                new_goals_publisher.publish(new_goals_msg)
                print("New goals published")
                crtl_c = True
            else:
                rospy.Rate(1).sleep()
        


############### END ENERGY CALLBACK ######################

############### CHECK NEW GOALS IN THE MAP ############################
def check_new_goals(image_rgb):
    new_goals = []
    px_to_en = []
    check_neigh = np.zeros([image_rgb.shape[0],image_rgb.shape[1]])
    # find neighbourhood 
    for i in range(0, image_rgb.shape[0]):
        for j in range(0, image_rgb.shape[1]):
            check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])
            if ((check_tuple != (0,0,0) and check_tuple != (0,0,255)) and check_neigh[i,j] != 1):  ## pixel to be energized
                px_to_en.append(i)  #Append x to be transformed
                px_to_en.append(j)  #Append y to be transformed
                check_neigh[i,j] = 1  #it is considered to be energized

                #find neighbourhood
                for k in range(j-1, j-8,-1): # trhee pixels left
                    check_tuple = (image_rgb[i,k,0], image_rgb[i,k,1], image_rgb[i,k,2])
                    if check_tuple != (0,0,0) and check_tuple != (0,0,255):  ## pixel to be energized
                        check_neigh[i,k] = 1
                    else:
                        break
                
                for k in range(j+1, j+8): # trhee pixels right
                    check_tuple = (image_rgb[i,k,0], image_rgb[i,k,1], image_rgb[i,k,2])
                    if check_tuple != (0,0,0) and check_tuple != (0,0,255):  ## pixel to be energized
                        check_neigh[i,k] = 1
                    else:
                        break
                
                for k in range(i-1, i-8,-1): # trhee pixels up
                    check_tuple = (image_rgb[k,j,0], image_rgb[k,j,1], image_rgb[k,j,2])
                    if check_tuple != (0,0,0) and check_tuple != (0,0,255):  ## pixel to be energized
                        check_neigh[k,j] = 1
                    else:
                        break
                
                for k in range(i+1, i+8): # trhee pixels down
                    check_tuple = (image_rgb[k,j,0], image_rgb[k,j,1], image_rgb[k,j,2])
                    if check_tuple != (0,0,0) and check_tuple != (0,0,255):  ## pixel to be energized
                        check_neigh[k,j] = 1
                    else:
                        break

    # find new goals based on pixel found in previous loops

    new_goals = []
    for i in range(0, len(px_to_en),2):
        x_px_en = px_to_en[i]
        y_px_en = px_to_en[i+1]
        check_goal = False # bool used to indicate that a goal has already been identified among neighbour pixels

        limit_up = x_px_en - 6
        if limit_up < 0:
            limit_up = 0

        limit_left = y_px_en - 6
        if limit_left < 0:
            limit_left = 0

        limit_right = y_px_en + 6
        if limit_right >= image_rgb.shape[1]-1:
            limit_right = image_rgb.shape[1]

        limit_down = x_px_en + 6
        if limit_down >= image_rgb.shape[0]-1:
            limit_down = image_rgb.shape[0] 

        
        for k in range(y_px_en-1,limit_left,-1): # scan 5 pixel to the left
            check_tuple = (image_rgb[x_px_en,k,0], image_rgb[x_px_en,k,1], image_rgb[x_px_en,k,2])
            if check_tuple == (0,0,0):
                break
            elif check_tuple != (0,0,0) and k == y_px_en-5:
                new_goals.append(x_px_en)
                new_goals.append(k)
                check_goal = True

        for k in range(y_px_en+1,limit_right): # scan 5 pixel to the right
            check_tuple = (image_rgb[x_px_en,k,0], image_rgb[x_px_en,k,1], image_rgb[x_px_en,k,2])
            if check_tuple == (0,0,0) or check_goal is True:
                break
            elif check_tuple != (0,0,0) and k == y_px_en+5:
                new_goals.append(x_px_en)
                new_goals.append(k)
                check_goal = True

        for k in range(x_px_en-1,limit_up,-1): # scan 5 pixel up
            check_tuple = (image_rgb[k,y_px_en,0], image_rgb[k,y_px_en,1], image_rgb[k,y_px_en,2])
            if check_tuple == (0,0,0) or check_goal is True:
                break
            elif check_tuple != (0,0,0) and k == x_px_en-5:
                new_goals.append(k)
                new_goals.append(y_px_en)
                check_goal = True

        for k in range(x_px_en+1,limit_down): # scan 5 pixel down
            check_tuple = (image_rgb[k,y_px_en,0], image_rgb[k,y_px_en,1], image_rgb[k,y_px_en,2])
            if check_tuple == (0,0,0) or check_goal is True:
                break
            elif check_tuple != (0,0,0) and k == x_px_en+5:
                new_goals.append(k)
                new_goals.append(y_px_en)
                check_goal = True

        if check_goal is False:
            rospy.logwarn("check_goal is False!")
        check_goal = False # put False for the next iteration 

    print(len(new_goals))
    print(new_goals)
    return new_goals




################### UP LEFT ######################
def up_left(x_px, y_px,image_rgb, energy_matrix):
    # First quadrant (up-left)
    global dt 
    global energy_const
    global dist_

    limit_up = x_px - dist_
    if limit_up < 0:
        limit_up = 0

    limit_left = y_px - dist_
    if limit_left < 0:
        limit_left = 0

    for i in range(x_px, limit_up,-1): #up
        for j in range(y_px -1, limit_left,-1): #left
            check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])

            if check_tuple == (0,0,0) and i == x_px:
                break # go in the next (up) row

            elif check_tuple == (0,0,0): #if the pixel is black
                for k in range(i, limit_up, -1): # Decreasing range 
                    energy_matrix[k,j,1] = 1 
                break # go in the next (up) row
            
            elif energy_matrix[i,j,1] == 1:
                break # go in the next (up) row

            energy_old = energy_matrix[i,j,0]
            energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)

            # compute colors: as the energy increases the blue and green decreases
            colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            if colorB <= 0:
                colorB = 0
            if colorG <= 0:
                colorG = 0

            image_rgb[i,j] = [colorB,colorG, 255]

    return image_rgb, energy_matrix

#####################À UP RIGHT #########################À
def up_right(x_px, y_px,image_rgb, energy_matrix):
    # Second quadrant (up-right)
    global dt 
    global energy_const
    global dist_

    limit_up = x_px - dist_
    if limit_up < 0:
        limit_up = 0

    limit_right = y_px + dist_
    if limit_right >= energy_matrix.shape[1]-1:
        limit_right = energy_matrix.shape[1]

    for i in range(x_px, limit_up,-1): # up
        for j in range(y_px + 1, limit_right): # right
            check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])

            if check_tuple == (0,0,0) and i == x_px:
                break # go in the next (up) row

            elif check_tuple == (0,0,0): #if the pixel is black
                for k in range(i, limit_up,-1):
                    energy_matrix[k,j,1] = 1 
                break # go in the next (up) row

            elif energy_matrix[i,j,1] == 1:
                break # go in the next (up) row

            energy_old = energy_matrix[i,j,0]
            energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)
            # compute colors: as the energy increases the blue and green decreases
            colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            if colorB <= 0:
                colorB = 0
            if colorG <= 0:
                colorG = 0

            image_rgb[i,j] = [colorB,colorG, 255] # the visualization is opposite wrt code: visualization is bgr, image_rgb is rgb
    return image_rgb, energy_matrix

##########################à DOWN RIGHT #######################
def down_right(x_px, y_px,image_rgb, energy_matrix):
    # Third quadrant (down-right)
    global dt 
    global energy_const
    global dist_

    limit_down = x_px + dist_
    if limit_down >= energy_matrix.shape[0]-1:
        limit_down = energy_matrix.shape[0]

    limit_right = y_px + dist_
    if limit_right >= energy_matrix.shape[1]-1:
        limit_right = energy_matrix.shape[1]


    for i in range(x_px + 1, limit_down): # down part
        for j in range(y_px + 1, y_px + dist_): # right
            check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])

            if check_tuple == (0,0,0): #if the pixel is black
                for k in range(i, limit_down):
                    energy_matrix[k,j,1] = 1 
                break # go in the next (down) row

            elif energy_matrix[i,j,1] == 1:
                break 

            energy_old = energy_matrix[i,j,0]
            energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)

            # compute colors: as the energy increases the blue and green decreases
            colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            if colorB <= 0:
                colorB = 0
            if colorG <= 0:
                colorG = 0

            image_rgb[i,j] = [colorB,colorG, 255]
    return image_rgb, energy_matrix

################ DOWN LEFT #################
def down_left(x_px, y_px,image_rgb, energy_matrix):
    # fourth quadrant (down-left)
    global dt 
    global energy_const
    global dist_

    limit_down = x_px + dist_
    if limit_down >= energy_matrix.shape[0]-1:
        limit_down = energy_matrix.shape[0]

    limit_left = y_px - dist_
    if limit_left < 0:
        limit_left = 0

    for i in range(x_px + 1, limit_down): #down
        for j in range(y_px - 1 , limit_left,-1): #left
            check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])

            if check_tuple == (0,0,0): #if the pixel is black
                for k in range(i, limit_down):
                    energy_matrix[k,j,1] = 1 
                break # go in the next (down) row

            elif energy_matrix[i,j,1] == 1:
                break 

            energy_old = energy_matrix[i,j,0]
            energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)
            # compute colors: as the energy increases the blue and green decreases
            colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
            if colorB <= 0:
                colorB = 0
            if colorG <= 0:
                colorG = 0

            image_rgb[i,j] = [colorB,colorG, 255]
    return image_rgb, energy_matrix


################ ONLY UP ###############
def only_up(x_px, y_px,image_rgb, energy_matrix):
    ## ONLY UP
    global dt 
    global energy_const
    global dist_
    j = y_px

    limit_up = x_px - dist_
    if limit_up < 0:
        limit_up = 0

    for i in range(x_px-1, limit_up, -1):
        check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])
        if check_tuple == (0,0,0): #if the pixel is black
            break # go in the next (up) row

        energy_old = energy_matrix[i,j,0]
        energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)

        # compute colors: as the energy increases the blue and green decreases
        colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
        colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
        if colorB <= 0:
            colorB = 0
        if colorG <= 0:
            colorG = 0

        image_rgb[i,j] = [colorB,colorG, 255]

    return image_rgb, energy_matrix

############### ONLY DOWN ####################
def only_down(x_px, y_px,image_rgb, energy_matrix):
    global dt 
    global energy_const
    global dist_
    j = y_px

    limit_down = x_px + dist_
    if limit_down >= energy_matrix.shape[0] - 1:
        limit_down = energy_matrix.shape[0]
    

    for i in range(x_px+1, limit_down):
        check_tuple = (image_rgb[i,j,0], image_rgb[i,j,1], image_rgb[i,j,2])
        if check_tuple == (0,0,0): #if the pixel is black
            break # go in the next (up) row

        energy_old = energy_matrix[i,j,0]
        energy_matrix[i,j,0] = energy_old + (energy_const* dt)/((i-x_px)**2+(j-y_px)**2)

        # compute colors: as the energy increases the blue and green decreases
        colorB = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
        colorG = int(round(translate(energy_matrix[i,j,0], 0, 0.01, 255, 0)))
        if colorB <= 0:
            colorB = 0
        if colorG <= 0:
            colorG = 0

        image_rgb[i,j] = [colorB,colorG, 255]

    return image_rgb, energy_matrix


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


################# MAIN ######################
if __name__ == '__main__':

    global image_rgb
    global dt
    global energy_const
    global energy_matrix
    global dist_
    global goals_finished


    goals_finished = False
    dt = 1
    energy_const = 0.1
    dist_ = 12 # pixels

    image = cv2.imread('/home/lorenzo/map_black_resized.png')
    image_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

    rospy.loginfo("Images imported successfully!")
    print(image_rgb.shape)

    energy_matrix = np.zeros([image_rgb.shape[0],image_rgb.shape[1],2]) # create a matrix with the same dimensions of the image
    print(energy_matrix.shape)
    # energy_matrix is a matrix in which the first channel represents the power level whereas the second is a sort of bool used 
    # to indicate when the pixel can be energized or not (if it is covered by a wall or is a wall itself it cannot be energized)

    rospy.init_node('energy_cv', anonymous=True)
    rospy.Subscriber("goals_finished", Bool, goals_finished_callback)
    rospy.Subscriber("robot_pose", PoseStamped,  energy_callback)
    new_goals_publisher = rospy.Publisher("new_goals", Array, queue_size=10)
    rospy.loginfo("ENERGY NODE")
    rospy.spin()
    #listener()
