#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Constants for the occupancy gridd
occupancy_grid_size = 200
resolution = .1

counter = 0
scale = -1
divider = 10

grass_image = True

bridge = CvBridge()
image_pub = rospy.Publisher("/igvc/lane_map",OccupancyGrid, queue_size=10)


# returns a filtered image and unfiltered image. This is needed for white lines on green grass
# output are two images, First output is the filtered image, Second output is the original pre-filtered image
def grass_filter(og_image):
    kernel = np.ones((3, 3), np.uint8)

    result = og_image.copy()
    img = cv2.cvtColor(og_image, cv2.COLOR_BGR2HSV)
    # create a lower bound for a pixel value
    lower = np.array([0, 0, 200])
    # create an upper bound for a pixel values
    upper = np.array([179, 77, 255])
    # detects all white pixels wihin the range specified earlier
    mask = cv2.inRange(img, lower, upper)
    result = cv2.bitwise_and(result, result, mask=mask)

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        area = cv2.contourArea(c)
        if area < 1:
            cv2.drawContours(result, [c], -1, (0, 0, 0), -1)

    gradient = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)

    return result, og_image

# takes in an image and outputs an image that has redlines overlaying the detected boundries
def camera_callback(data):

    image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    # convert the image to a better RGB values
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # flag for seeing if you're dealing with grassy images or not
    if grass_image:
        # for white lines on grassy photos need this part for pre-processing before being able to detect white lines
        img_list = grass_filter(image)
        pre_or_post_filtered_image = img_list[0]
        original_overlay_image = img_list[1]
    else:
        # need this if testing on actual roads with white lines borders
        pre_or_post_filtered_image = image
        original_overlay_image = image

    # gives the height and width of the image from the dimensions given
    height = image.shape[0]
    width = image.shape[1]

    # curvy road test width-50
    # region_of_interest_vertices = [
    #     (0, height),
    #     (width / 2 - 10, height / 2 + 50),
    #     (width - 50, height),
    # ]

    # using for the hd highway video to for propper crop
    # region_of_interest_vertices = [
    #     (300, height),
    #     (width / 2 + 100, height / 2 + 300),
    #     (width * .8, height),
    # ]

    # used for non-hd video
    region_of_interest_vertices = [
        (0, height),
        (width / 2, height / 2 + 70),
        (width, height),
    ]

    # convert to grayscale
    gray_image = cv2.cvtColor(pre_or_post_filtered_image, cv2.COLOR_RGB2GRAY)

    # call Canny Edge Detection
    cannyed_image = cv2.Canny(gray_image, 100, 200)

    # crop operation at the end of the cannyed pipeline so cropped edge doesn't get detected
    cropped_image = region_of_interest(cannyed_image, np.array([region_of_interest_vertices], np.int32))

    # used houghlinesP algo to detect the white lines
    # use threshold=152 for road side image. Test out different stuff for grassy images
    lines = cv2.HoughLinesP(cropped_image, rho=6, theta=np.pi / 60, threshold=75,
                            lines=np.array([]), minLineLength=40, maxLineGap=25)

    # create an occupancy grid from the given lines and the height and width of the image
    map_localization(lines, width, height)



# takes in an image and a list of points (vertices)matplotlib.use('agg')to crop the image
def region_of_interest(img, vertices):
    # define a blank matrix that matches the iamge matplotlib.use('agg')eight/width.
    mask = np.zeros_like(img)

    # Create a match color for gray scalled images
    match_mask_color = 255

    # Fill inside the polygon
    cv2.fillPoly(mask, vertices, match_mask_color)

    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image


# the if statement that determines what is left or right lane will need to change based on video footage
def map_localization(lines, width, height):
    global occupancy_grid_size
    global scale
    global divider

    x_left_list = []
    x_right_list = []
    y_left_list = []
    y_right_list = []

    x_left_list_r = []
    x_right_list_r = []
    y_left_list_r = []
    y_right_list_r = []

    # r stands for rounded value
    # I add 50 to guarantee that it will round up to the neared hundreds
    width_r = int(round_up(width, scale))
    height_r = int(round_up(height, scale))
    height_r = height_r / divider
    width_r = width_r / divider

    # error checking to see that if lines do in fact exists
    try:
        # populates two lists with x and y values
        for line in lines:
            for x1, y1, x2, y2 in line:
                # if the x location of the pixel is less than 500 then it's the left line
                # this if statement will need to change based on video footage
                if x1 < 500:
                    x_left_list.extend([x1, x2])
                    y_left_list.extend([y1, y2])
                else:
                    x_right_list.extend([x1, x2])
                    y_right_list.extend([y1, y2])

        # makes sure the list isn't empty before trying to round
        # creating np arrays from original right and left line lists
        # then using np's built in functions to round
        # and turning them back into normal lists
        # work on this ,maybe go back to what i had
        if len(x_right_list) > 0:
            x_right_list_r = list(np.array(x_right_list) / divider)
            # adding half the height for the occupancy grid
            y_right_list_r = list((np.array(y_right_list) / divider))

        if len(x_left_list) > 0:
            x_left_list_r = list(np.array(x_left_list) / divider)
            # adding half the height for the occupancy grid
            y_left_list_r = list((np.array(y_left_list) / divider))

        # creates a 2d array of widthxheight (in this particular case) (row x column)
        data_map = np.zeros(shape=(int(height_r), int(width_r)), dtype=int)

        # error checking
        if len(x_right_list) > 0:
            # gets the average of the x on the left and right sides
            # I'm doing this so I can get a consistent line
            x_right_list_avg = np.mean(x_right_list_r)
            # loops through the x and y coordinates and places a 1 on the map representing the line from the image
            for row in y_right_list_r:
                data_map[int(row)][int(x_right_list_avg)] = 1

                for i in range(4):
                    # checks i below and above to fill in any gaps that might have been missed
                    if 0 < int(row) - i and data_map[int(row)][int(x_right_list_avg)] == 1:
                        data_map[int(row) - i][int(x_right_list_avg)] = 1

                    if int(row) + i < height_r and data_map[int(row)][int(x_right_list_avg)] == 1:
                        data_map[int(row) + i][int(x_right_list_avg)] = 1

        # populates left side of the map
        if len(x_left_list) > 0:
            x_left_list_avg = np.mean(x_left_list_r)
            for row in y_left_list_r:
                data_map[int(row)][int(x_left_list_avg)] = 1

                for i in range(4):
                    # checks i below and above to fill in any gaps that might have been missed
                    if 0 < int(row) - i and data_map[int(row)][int(x_left_list_avg)] == 1:
                        data_map[int(row) - i][int(x_left_list_avg)] = 1

                    if int(row) + i < height_r and data_map[int(row)][int(x_left_list_avg)] == 1:
                        data_map[int(row) + i][int(x_left_list_avg)] = 1
    # if there is a null error then set the data_map to be empty
    except TypeError:
        data_map = np.zeros(shape=(int(height_r), int(width_r)), dtype=int)

    # resizes the image to so the lines are passed the half the height
    data_map_resized = cv2.resize(data_map, dsize=(occupancy_grid_size, occupancy_grid_size), interpolation=cv2.INTER_NEAREST)
    # shifts the map by 100, which is where the robot is centered at
    data_map_resized = np.roll(data_map_resized, 100, axis=0)

    # make 1d array to list after 'flattening it' to 1d
    data_map_resized = list(data_map_resized.flatten())

    numpy_to_occupancyGrid(data_map_resized)
    

def numpy_to_occupancyGrid(data_map):
    msg = OccupancyGrid(info=MapMetaData(width=occupancy_grid_size,height=occupancy_grid_size,resolution=.1), data = data_map)
    image_pub.publish(msg)

    # created my own helper function to round up numbers
def round_up(n, decimals):
    multiplier = 10 ** decimals
    return math.ceil(n * multiplier) / multiplier

if __name__ == "__main__":
    rospy.init_node('lane_finder', anonymous=True)
    # Need to subscribe to an image node for images data to use
    rospy.Subscriber("/cv_camera/image_raw", Image, camera_callback)
    # while true loop
    rospy.spin()
