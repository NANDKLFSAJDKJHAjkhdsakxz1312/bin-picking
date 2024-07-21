#!/usr/bin/env python3

import rospy
import random
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np

def random_position(min_val, max_val):
    return random.uniform(min_val, max_val)

def random_rotation(max_angle):
    return random.uniform(-max_angle, max_angle)

def move_checkerboard():
    rospy.init_node('move_checkerboard_random_with_rotation')

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        while not rospy.is_shutdown():
            state_msg = ModelState()
            state_msg.model_name = 'checkerboard_9_7_0_1'  # 棋盘模型的名称

            # 生成随机位置
            state_msg.pose.position.x = random_position(-1.2, -0.2)
            state_msg.pose.position.y = random_position(0.3, 1.2)
            state_msg.pose.position.z = random_position(1.1,1.6)

            # 生成随机旋转（正负0.05弧度以内）
            roll = random_rotation(1)
            pitch = random_rotation(1)
            yaw = random_rotation(1)

            # 转换为四元数
            qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

            state_msg.pose.orientation.x = qx
            state_msg.pose.orientation.y = qy
            state_msg.pose.orientation.z = qz
            state_msg.pose.orientation.w = qw

            resp = set_state(state_msg)
            if resp.success:
                rospy.loginfo("Moved checkerboard successfully")
            else:
                rospy.logwarn("Failed to move checkerboard")
            
            rospy.sleep(2)  # 等待2秒后移动到下一个位置

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角转换为四元数
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

if __name__ == '__main__':
    try:
        move_checkerboard()
    except rospy.ROSInterruptException:
        pass
