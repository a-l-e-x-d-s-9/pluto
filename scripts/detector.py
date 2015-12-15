#!/usr/bin/env python

import roslib
import rospy
import cv2
import numpy as np
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pluto.msg import DetectResult


class Detector:
    sample_counter          = 0
    image_listener          = 0
    image_cv                = 0
    active_image_capture    = True
    detect_result_publisher = 0
    
    
    def DETECTOR_RATE( self ):
        return 5
        
    def DETECTOR_IMAGE_WIDTH( self ):
        return 640
        
    def DETECTOR_IMAGE_HEIGHT( self ):
        return 480
        
    def DETECTOR_DISCRETE_TURN_PIXELS( self ):
        return 120

    def __init__( self ):
        rospy.loginfo( "Detector innitialized " )
        self.image_listener = rospy.Subscriber("/komodo_1/Asus_Camera/rgb/image_raw", Image, self.rgb_listener_cb)
        rospy.Subscriber("pluto/detect/command", String, self.detect_ball )
        self.detect_result_publisher = rospy.Publisher('pluto/detect/result', DetectResult, queue_size=10 )

    def rgb_listener_cb( self, rgb_image ):
        if True == self.active_image_capture:
            
            #rospy.loginfo( "rgb_listener_cb, {}".format(self.sample_counter) )
            bridge = cv_bridge.CvBridge()

            try:
                self.image_cv = bridge.imgmsg_to_cv2( rgb_image, "bgr8" )

            except cv_bridge.CvBridgeError, cv_bridge_except:
                rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))
                return
            
            #if 10 == self.sample_counter:
            #    cv2.imshow( "rgb", self.image_cv )
            #    cv2.waitKey()
            
            self.sample_counter = self.sample_counter + 1

    def find_the_ball( self, image_cv, debug = False ):
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
        
        
    def calculate_turns_needed( self, center_coordinates_and_radius ):
        (x, y, r) = center_coordinates_and_radius
        
        turns = ( x // self.DETECTOR_DISCRETE_TURN_PIXELS() ) - ( self.DETECTOR_IMAGE_WIDTH() // self.DETECTOR_DISCRETE_TURN_PIXELS() // 2 )
        
        return turns


    def detect_ball( self, string_command ):
        
        detect_result = DetectResult()
        detect_result.is_ball_detected = False
        
        if self.sample_counter > 0:
            
            image_height, image_width = self.image_cv.shape[:2]
            #image_width, image_height = cv2.GetSize(self.image_cv)
            
            rospy.loginfo( "image_width: {}, image_height: {}".format( image_width, image_height ) )
            
            process_image = (image_width == self.DETECTOR_IMAGE_WIDTH()) and (image_height == 480)
            
            if True == process_image:
                is_ball_found, center_coordinates_and_radius = self.find_the_ball( self.image_cv, debug = False )
                
                detect_result.is_ball_detected = is_ball_found
                
                if True == is_ball_found:
                    detect_result.discrete_turns_needed = self.calculate_turns_needed( center_coordinates_and_radius )
                    
        self.detect_result_publisher.publish( detect_result )
        
if __name__ == '__main__':
    try:
        rospy.init_node("pluto_detector")
        detector = Detector()
        
        #rate = rospy.Rate( detector.DETECTOR_RATE() )
        
        while not rospy.is_shutdown():
            
            rospy.spin()
            #detector.detect_ball()
            
            #rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
