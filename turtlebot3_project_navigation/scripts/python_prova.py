#!/usr/bin/env python

'''
23 Jan 2022
'''
import cv2
import numpy as np

'''
def listener():

    image = cv2.imread('/home/lorenzo/map_black_resized.png')
    image_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    print(image.shape)
    image_rgb[55,0] = [0, 255, 0] # draw green pixel on the position of the robot
    cv2.imshow('image',image_rgb)
    #cv2.waitKey(0)
    cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
    cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
    cv2.imshow('image_rsz',image_rgb) # display small image in big window
    cv2.waitKey(0)


if __name__ == '__main__':
    listener()
    #cv2.destroyAllWindows()

'''


'''
if __name__ == '__main__':
    with open("/home/lorenzo/turtle_ws/src/turtlebot3_project_navigation/scripts/goals.txt", "r") as file:
        lines = file.readlines()
        for line in lines:
            goal = line.split()
            print(goal)
            goal_x = float(goal[0])
            goal_y = float(goal[1])
            print(goal_x)
            print(goal_y)
'''


'''
image = cv2.imread('/home/lorenzo/map_black.png')
# convert the image to grayscale format
img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print(img_gray.shape)

# perform erosion and dilation in order to "separate" the rooms
kernel = np.ones((3,3),np.uint8)
erosion = cv2.erode(img_gray,kernel,iterations = 10)
dilate = cv2.dilate(erosion,kernel,iterations = 10)

# Show the updated image 
#cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
#cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
#cv2.imshow('image_rsz',dilate) # display small image in big window
#cv2.waitKey(0)

# detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
contours, hierarchy = cv2.findContours(image=dilate, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
# convert to rgb (just for visualization)
image_rgb = cv2.cvtColor(dilate,cv2.COLOR_BGR2RGB)

n_rooms = 0
for contour in contours: 
    x,y,w,h = cv2.boundingRect(contour)
    area = w*h
    if area > 150:
        n_rooms+=1       
        cv2.rectangle(image_rgb,(x,y),(x+w,y+h),(0,0,255),2)


print("In this house I found", n_rooms, "rooms.")    

# Show the updated image 
cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
cv2.imshow('image_rsz',image_rgb) # display small image in big window
cv2.waitKey(0)
'''

image = cv2.imread('/home/lorenzo/map_from_code.png')
image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

pixel = image[60,20]
print(pixel)
cv2.namedWindow('image_rsz',cv2.WINDOW_NORMAL) # create a window for the visualization of a image
cv2.resizeWindow('image_rsz', 800,800) # resize the window to display a small image in a big window
cv2.imshow('image_rsz',image) # display small image in big window
cv2.waitKey(0)