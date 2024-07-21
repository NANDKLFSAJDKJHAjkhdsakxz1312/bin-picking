#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
import argparse

def control_gripper(state):
    service_name = '/robot_with_arm_and_camera/on' if state else '/robot_with_arm_and_camera/off'
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Empty)
        resp = service_proxy()
        rospy.loginfo("Service %s called successfully", service_name)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("gripper_controller", anonymous=False)

    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Control the vacuum gripper.')
    parser.add_argument('action', choices=['on', 'off'], help='Action to perform on the gripper (on/off)')
    args = parser.parse_args()

    # 提示用户操作
    rospy.loginfo("Starting gripper control node...")
    if args.action == 'on':
        rospy.loginfo("Turning on the vacuum gripper...")
        control_gripper(True)  # 调用服务以开启吸盘
        rospy.loginfo("Vacuum gripper is now ON.")
    elif args.action == 'off':
        rospy.loginfo("Turning off the vacuum gripper...")
        control_gripper(False)  # 调用服务以关闭吸盘
        rospy.loginfo("Vacuum gripper is now OFF.")
    else:
        rospy.logwarn("Invalid action. Please use 'on' or 'off'.")

    rospy.loginfo("Gripper control node is now spinning. Press Ctrl+C to exit.")
    rospy.spin()
