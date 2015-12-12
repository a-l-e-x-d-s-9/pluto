#!/usr/bin/env python

import rospy
        
if __name__ == '__main__':
    try:
        rospy.init_node("pluto_detector")

        rate = rospy.Rate( 20 )
        while not rospy.is_shutdown():

            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
