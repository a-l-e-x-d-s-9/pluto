#!/usr/bin/env python

import cv2
import numpy as np


def find_the_ball( image_cv, debug = False ):
    # return: 
    #           1) is_ball_found - boolean 
    #           2) center_coordinates_and_radius - tuple (x,y,r)
    # Algorithm: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
    
    image_height, image_width = image_cv.shape[:2]

    hsv_image = cv2.cvtColor( image_cv, cv2.COLOR_BGR2HSV );

    # 25 to 90 is approximated H values for yellow, 85 to 255 is S and V values we use
    yellow_hue_range = cv2.inRange( hsv_image, (25, 85, 85), (90, 255, 255) )

    yellow_hue_range_blured = cv2.GaussianBlur(yellow_hue_range, (11, 11), 2, 2);

    # http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=houghcircles#houghcircles
    circles = cv2.HoughCircles( yellow_hue_range_blured, cv2.HOUGH_GRADIENT, 1, 20, param1=100,param2=4,minRadius=0,maxRadius=0)
    #circles = cv2.HoughCircles( yellow_hue_range_blured, cv2.HOUGH_GRADIENT, 1, image_height/8, param1=100,param2=20,minRadius=0,maxRadius=0);
    #param1=50,param2=30,minRadius=0,maxRadius=0
    #param1=100,param2=20,minRadius=0,maxRadius=0

    if True == debug:
        cv2.imshow( "lower_yellow_hue_range", yellow_hue_range_blured )
        cv2.waitKey()

    is_ball_found                   = circles is not None
    center_coordinates_and_radius   = (0, 0, 0) 

    if True == is_ball_found:
        circles = np.uint16(np.around(circles))
        
        if True == debug:
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(image_cv,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(image_cv,(i[0],i[1]),2,(0,0,255),3)

            cv2.imshow( "lower_yellow_hue_range", image_cv )
            cv2.waitKey()
            
        center_coordinates_and_radius = circles[0][0]
        
    return is_ball_found, center_coordinates_and_radius
        

bgr_image = cv2.imread('/home/alexds9/catkin_ws/src/pluto/scripts/image.png',1)
is_ball_found, center_coordinates_and_radius = find_the_ball( bgr_image, debug = True )

print is_ball_found
print center_coordinates_and_radius
