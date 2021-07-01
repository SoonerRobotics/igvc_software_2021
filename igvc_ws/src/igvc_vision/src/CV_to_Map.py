#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rospy
import time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image


occupancy_grid_size = 200

start_time = None

# Output map scale and offset
vertical_offset = 5
height_offset = 58
captured_width = 70
captured_height = 50

frame_rate = 14 # Hz

preview_pub = rospy.Publisher("/igvc/preview", Image, queue_size=1)
image_pub = rospy.Publisher("/igvc/lane_map", OccupancyGrid, queue_size=1)

header = Header()
header.frame_id = "base_link"

map_info = MapMetaData()
map_info.width = occupancy_grid_size
map_info.height = occupancy_grid_size
map_info.resolution = 0.1
map_info.origin = Pose()
map_info.origin.position.x = -10
map_info.origin.position.y = -10

cam = None

class PerspectiveTransform:

    def __init__(self, camera_angle):
        self.top_trim_ratio = 0.0
        
        # Camera angle in degrees, where 0 is when camera is facing perpendicular to the ground
        self.camera_angle = camera_angle

        # Ratio of number of pixels between top points of the trapezoid and their nearest vertical border
        self.horizontal_corner_cut_ratio = 0.3

        # Output image dimensions
        self.output_img_shape_x = 640
        self.output_img_shape_y = 480
 
    # Equation to calculate how much off the top to trim. Should probably edit this once we have the robot built
    '''
    Calculates how much of the top portion of the image needs to be trimmed
    return: ratio of amount to be trimmed and total
    '''
    def calc_trim_ratio(self):
        return (1/4 * self.camera_angle) / 100
    

    '''
    Trim top portion of the image
    img: Raw image obtained from camera
    return: Trimmed image
    '''
    def trim_top_border(self, img):
        y_min = (int)(img.shape[0] * self.calc_trim_ratio())
        y_max = (int)(img.shape[0])
        x_min = 0
        x_max = (int)(img.shape[1])

        return img[y_min:y_max, x_min:x_max]
    
    '''
    Convert angled image into a top down viewed image
    img: Cropped image
    return: Top down image
    '''
    def convert_to_flat(self, img):
        
        # Define the trapezoid to transform into rectangle
        top_left = (int)(img.shape[1] * self.horizontal_corner_cut_ratio), (int)(img.shape[0])
        top_right = (int)(img.shape[1] - img.shape[1] * self.horizontal_corner_cut_ratio), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src_pts = np.float32([[top_left], [top_right], [bottom_left], [bottom_right]])
        dest_pts = np.float32([ [0, self.output_img_shape_y], [self.output_img_shape_x, self.output_img_shape_y] ,[0, 0], [self.output_img_shape_x, 0]])

        matrix = cv2.getPerspectiveTransform(dest_pts, src_pts)
        output = cv2.warpPerspective(img, matrix, (self.output_img_shape_x, self.output_img_shape_y))

        return output

transform = PerspectiveTransform(5)

# returns a filtered image and unfiltered image. This is needed for white lines on green grass
# output are two images, First output is the filtered image, Second output is the original pre-filtered image
def grass_filter(og_image):
    img = cv2.cvtColor(og_image, cv2.COLOR_BGR2HSV)
    # create a lower bound for a pixel value
    lower = np.array([0, 0, 100])
    # create an upper bound for a pixel values
    upper = np.array([255, 80, 200])
    # detects all white pixels wihin the range specified earlier
    mask = cv2.inRange(img, lower, upper)
    mask = 255 - mask

    return mask


# takes in an image and outputs an image that has redlines overlaying the detected boundries
def camera_callback(data):
    global img_num, start_time

    # start_time = time.time()

    read_success, image = cam.read()
    # print(f"read_success: {read_success}, read time: {(time.time() - start_time) * 1000:02.02f}ms")

    if not read_success:
        return

    image = cv2.GaussianBlur(image, (7,7), 0)
    image = cv2.GaussianBlur(image, (7,7), 0)

    pre_or_post_filtered_image = grass_filter(image)

    # gives the height and width of the image from the dimensions given
    height = image.shape[0]
    width = image.shape[1]

    # used for non-hd video
    region_of_interest_vertices = [
        (0, height),
        (width / 2, height / 2 + 70),
        (width, height),
    ]

    # convert to grayscale
    # gray_image = cv2.cvtColor(pre_or_post_filtered_image, cv2.COLOR_RGB2GRAY)
    gray_image = pre_or_post_filtered_image

    # crop operation at the end of the cannyed pipeline so cropped edge doesn't get detected
    cropped_image = region_of_interest(gray_image, np.array([region_of_interest_vertices], np.int32))

    blurred = cv2.GaussianBlur(cropped_image, (7, 7), 0)
    blurred[blurred < 245] = 0

    perpsective_crop = transform.trim_top_border(blurred)
    perspective_warp = transform.convert_to_flat(perpsective_crop)

    # publishes to the node
    numpy_to_occupancyGrid(perspective_warp)
    # print(f"CV time: {(end_time - start_time) * 1000:02.02f}ms")


# takes in an image and a list of points (vertices) to crop the image
def region_of_interest(img, vertices):
    # define a blank matrix that matches the iamge height/width.
    mask = np.ones_like(img) * 255

    # Create a match color for gray scalled images
    match_mask_color = 0

    # Fill inside the polygon
    cv2.fillPoly(mask, vertices, match_mask_color)

    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image

def numpy_to_occupancyGrid(data_map):
    # dsize is (width, height) and copyMakeBorder is (tp, bottom, left, right)
    data_map = cv2.dilate(data_map, (5, 5), iterations=3)
    data_map = cv2.resize(data_map, dsize=(captured_width, captured_height), interpolation=cv2.INTER_LINEAR) / 2
    data_map = data_map[vertical_offset:,:]
    data_map = cv2.copyMakeBorder(data_map, vertical_offset + height_offset, 200 - captured_height - height_offset, (200 - captured_width) // 2, (200 - captured_width) // 2, cv2.BORDER_CONSTANT, value=0)
    data_map = cv2.flip(data_map, 0)
    data_map = cv2.rotate(data_map, cv2.ROTATE_90_COUNTERCLOCKWISE)
    flattened = list(data_map.flatten().astype(int))

    # print(f"data_map shape: {data_map.shape}")

    header.stamp = rospy.Time.now()

    msg = OccupancyGrid(header=header, info=map_info, data=flattened)
    image_pub.publish(msg)

    
    # print(f"pub time: {(time.time() - start_time) * 1000:02.02f}ms")

# created my own helper function to round up numbers
def round_up(n, decimals):
    multiplier = 10 ** decimals
    return math.ceil(n * multiplier) / multiplier


if __name__ == '__main__':
    # call pipeline function which will return a data_map which is just a 2d numpy array
    # Need to subscribe to an image node for images data to use
    rospy.init_node('lane_finder', anonymous=True)
    # rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, camera_callback)
    
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    rospy.Timer(rospy.Duration(1.0/frame_rate), camera_callback)

# while true loop
    rospy.spin()
