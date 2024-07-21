#!/usr/bin/env python3

import rospy
import time

if __name__ == '__main__':
    rospy.init_node('delay_node')
    delay_time = rospy.get_param('~delay_time', 5)
    rospy.loginfo(f"Delaying for {delay_time} seconds...")
    time.sleep(delay_time)
    rospy.loginfo("Delay complete.")
