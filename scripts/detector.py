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
    sample_counter                      = 0
    top_camera_image_cv                 = 0
    arm_camera_image_cv                 = 0
    active_top_camera_image_capture     = True
    active_arm_camera_image_capture     = True
    detect_result_publisher             = 0
    
    
    def DETECTOR_RATE( self ):
        return 5
        
    def DETECTOR_IMAGE_WIDTH( self ):
        return 640
        
    def DETECTOR_IMAGE_HEIGHT( self ):
        return 480
        
    def __init__( self ):
        rospy.loginfo( "Detector innitialized " )
        rospy.Subscriber( "/Asus_Camera/rgb/image_raw",     Image, self.top_camera_listener_cb)
        rospy.Subscriber( "/Creative_Camera/rgb/image_raw", Image, self.arm_camera_listener_cb)
        
        rospy.Subscriber("pluto/detect/command", String, self.detect_ball )
        self.detect_result_publisher = rospy.Publisher('pluto/detect/result', DetectResult, queue_size=10 )

    def top_camera_listener_cb( self, rgb_image ):
        if True == self.active_top_camera_image_capture:
            
            #rospy.loginfo( "top_camera_listener_cb, {}".format(self.sample_counter) )
            bridge = cv_bridge.CvBridge()

            try:
                self.top_camera_image_cv = bridge.imgmsg_to_cv2( rgb_image, "bgr8" )

            except cv_bridge.CvBridgeError, cv_bridge_except:
                rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))
                return
            
            #if 10 == self.sample_counter:
            #    cv2.imshow( "rgb", self.top_camera_image_cv )
            #    cv2.waitKey()
            
            self.sample_counter = self.sample_counter + 1
            
    def arm_camera_listener_cb( self, rgb_image ):
        if True == self.active_arm_camera_image_capture:
            
            #rospy.loginfo( "top_camera_listener_cb, {}".format(self.sample_counter) )
            bridge = cv_bridge.CvBridge()

            try:
                self.arm_camera_image_cv = bridge.imgmsg_to_cv2( rgb_image, "bgr8" )

            except cv_bridge.CvBridgeError, cv_bridge_except:
                rospy.logerr("Failed to convert ROS image message to CvMat\n%s", str( cv_bridge_except ))
                return
            
            #if 10 == self.sample_counter:
            #    cv2.imshow( "rgb", self.top_camera_image_cv )
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
        circles = cv2.HoughCircles( yellow_hue_range_blured, cv2.cv.CV_HOUGH_GRADIENT, 1, 20, param1=100,param2=4,minRadius=0,maxRadius=0)
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


    def detect_ball( self, string_command ):
        
        detect_result = DetectResult()
        detect_result.is_ball_detected  = False
        detect_result.request_tag       = string_command.data
        
        if "scan_top" == detect_result.request_tag:
            working_image_cv = self.top_camera_image_cv
        elif "scan_arm" == detect_result.request_tag:
            working_image_cv = self.arm_camera_image_cv
        else:
            raise Exception('Unsupported command to detector, support only: \"scan_top\" or \"scan_arm\".')
        
        if self.sample_counter > 0:
            
            image_height, image_width = working_image_cv.shape[:2]
            
            #rospy.loginfo( "image_width: {}, image_height: {}".format( image_width, image_height ) )
            
            process_image = ( image_width == self.DETECTOR_IMAGE_WIDTH() ) and ( image_height == self.DETECTOR_IMAGE_HEIGHT() )
            
            if True == process_image:
                is_ball_found, center_coordinates_and_radius = self.find_the_ball( working_image_cv, debug = False )
                
                detect_result.is_ball_detected = is_ball_found
                
                if True == is_ball_found:
                    rospy.loginfo( "detect_ball found: x: {}, y: {}, r: {}".format( center_coordinates_and_radius[0], center_coordinates_and_radius[1], center_coordinates_and_radius[2] ) )
                    
                    detect_result.detected_x = center_coordinates_and_radius[0]
                    detect_result.detected_y = center_coordinates_and_radius[1]
                    detect_result.detected_r = center_coordinates_and_radius[2]
                else:
                    rospy.loginfo( "detect_ball not found")
        
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
