
import numpy as np
import cv2

import time

'''
To transform angled camera images into a top down image, the following steps are taken:
1. Trim the top border to ignore "far away objects"
2. Identify the shape (trapezoid) that will be transformed into a top down rectangle
3. Construct image from the shape
'''
class PerspectiveTransform:

    def __init__(self, camera_angle):
        self.top_trim_ratio = 0.15
        
        # Camera angle in degrees, where 0 is when camera is facing perpendicular to the ground
        self.camera_angle = camera_angle

        # Ratio of number of pixels between top points of the trapezoid and their nearest vertical border
        self.horizontal_corner_cut_ratio = 0.25

        # Output image dimensions
        self.output_img_shape_x = 640
        self.output_img_shape_y = 300
    
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






video = cv2.VideoCapture('test3.avi')

transform = PerspectiveTransform(70)

while (video.isOpened()):
    ret, frame = video.read()

    if (ret):
        cv2.imshow('frame', frame)

        cropped = transform.trim_top_border(frame)
        cv2.imshow('cropped', cropped)

        output = transform.convert_to_flat(cropped)
        cv2.imshow('output', output)


        time.sleep(0.005)  

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break 
    else:
        break
    

video.release()
cv2.destroyAllWindows()