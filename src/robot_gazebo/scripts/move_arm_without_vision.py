#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import moveit_commander
import geometry_msgs.msg
import sys
import threading
from std_srvs.srv import Empty

# 初始化MoveIt!的接口
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robot_controller', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("grasping")  # 替换为你的规划组名称

def move_arm_to_position(x, y, z):
    target_pose = geometry_msgs.msg.Pose()
    target_pose.orientation.w = 0
    target_pose.orientation.x = 1
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    
    group.set_pose_target(target_pose)
    
    plan = group.plan()
    if not plan:
        rospy.logerr("Failed to find a plan")
        return
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

def input_thread(pub):
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        command = input("Enter 'g' to grasp, 'r' to release, or 'm' to move: ")
        if command == 'g':
            pub.publish(True)
            rospy.loginfo("Grasp command sent")
            control_gripper(True)
        elif command == 'r':
            pub.publish(False)
            rospy.loginfo("Release command sent")
            control_gripper(False)
        elif command == 'm':
            x = float(input("Enter x position: "))
            y = float(input("Enter y position: "))
            z = float(input("Enter z position: "))
            move_arm_to_position(x, y, z)
            rospy.loginfo(f"Moving to position ({x}, {y}, {z})")
        rate.sleep()

def control_gripper(state):
    service_name = '/robot_with_arm_and_camera/vacuum_gripper/on' if state else '/robot_with_arm_and_camera/vacuum_gripper/off'
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Empty)
        resp = service_proxy()
        rospy.loginfo("Service %s called successfully", service_name)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('gripper_command', Bool, queue_size=10)
        
        # 启动用户输入线程
        thread = threading.Thread(target=input_thread, args=(pub,))
        thread.start()
        
        rospy.spin()  # 保持节点运行
        
        thread.join()  # 等待线程结束
    except rospy.ROSInterruptException:
        pass
