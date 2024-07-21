#!/usr/bin/env python3

import rospy
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped, Pose
from sensor_msgs.msg import CameraInfo, Image
import moveit_commander
import geometry_msgs.msg
import tf
from std_srvs.srv import Empty 
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from visualization_msgs.msg import Marker

# 初始化MoveIt!的接口
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("grasping")  # 替换为你的规划组名称

def undistort_points(x, y, camera_matrix, dist_coeffs):
    # 将像素坐标转为归一化坐标
    points = np.array([[x, y]], dtype=np.float32)
    points = np.expand_dims(points, axis=1)
    
    # 使用OpenCV函数进行畸变校正
    undistorted_points = cv2.undistortPoints(points, camera_matrix, dist_coeffs, P=camera_matrix)
    
    return undistorted_points[0][0]

class YoloToRobot:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub_boxes = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.boxes_callback)
        # self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/robot_with_arm_and_camera/depth_camera/depth/image_raw', Image, self.camera_callback)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.camera_info = None
        self.image = None
        self.dist_coeffs = None  # 初始化dist_coeffs


    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.dist_coeffs = np.array(msg.D) 
        # rospy.loginfo(f"Depth Camera Info: K={self.camera_info.K}")

    def camera_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # rospy.loginfo(f"Depth image encoding: {msg.encoding}")
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_depth(self, x, y):
        if self.image is not None and self.image.size > 0:
            return self.image[int(y), int(x)]
   

    def move_arm_to_position(self, x, y, z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation.w = 1.0
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        group.set_pose_target(target_pose)
        group.set_planning_time(10)  # 增加规划时间
        
        

        plan = group.plan()
        if not plan:
            rospy.logerr("Failed to find a plan")
            return
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    def control_gripper(self, state):
        service_name = '/robot_with_arm_and_camera/on' if state else '/robot_with_arm_and_camera/off'
        rospy.wait_for_service(service_name)
        try:
            service_proxy = rospy.ServiceProxy(service_name, Empty)
            resp = service_proxy()
            rospy.loginfo("Service %s called successfully", service_name)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def boxes_callback(self, msg):
        if not self.camera_info:
            rospy.logwarn("No camera info received yet")
            return
        if self.image is None or self.image.size == 0:
            rospy.logwarn("No depth image received yet")
            return

        for box in msg.bounding_boxes:
            if box.Class == 'box_block':
                x_center = (box.xmin + box.xmax) / 2
                y_center = (box.ymin + box.ymax) / 2

         
                z_depth = self.get_depth(x_center, y_center)
                if z_depth is None:
                    rospy.logwarn("No depth information available")
                    continue

                # 打印深度信息
                rospy.loginfo(f"Depth at ({x_center}, {y_center}) is {z_depth}")

                # 获取相机内参矩阵
                camera_matrix = np.array(self.camera_info.K).reshape(3, 3)
                dist_coeffs = np.array(self.dist_coeffs)
                print("dist_coeffs :", dist_coeffs)

                # 使用畸变系数进行校正
                undistorted_point = undistort_points(x_center, y_center, camera_matrix, dist_coeffs)
                x_center, y_center = undistorted_point

                cx = self.camera_info.K[2]
                cy = self.camera_info.K[5]
                fx = self.camera_info.K[0]
                fy = self.camera_info.K[4]

                x = (x_center - cx) * z_depth / fx
                y = (y_center - cy) * z_depth / fy
                z = z_depth

                # 打印相机坐标
                rospy.loginfo(f"depth_camera_link_frame coordinates: x={x}, y={y}, z={z}")

                point_camera = PointStamped()
                point_camera.header = msg.header
                
                point_camera.point.x = x
                point_camera.point.y = y
                point_camera.point.z = z

                try:
                    point_base = self.tf_listener.transformPoint('world', point_camera)
                    
                    # 打印转换后的base_point
                    rospy.loginfo(f"Transformed coordinates under world frame: x={point_base.point.x-2.13}, y={point_base.point.y}, z={3-z_depth+0.03}")
                    print("Frame ID of point_camera:", point_base.header.frame_id)

                    # 在RViz中可视化转换后的点
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "yolo_detections"
                    marker.id = 0
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = point_base.point.x
                    marker.pose.position.y = point_base.point.y
                    marker.pose.position.z = point_base.point.z
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.a = 1.0  # Don't forget to set the alpha!
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0

                    self.marker_pub.publish(marker)

                    self.move_arm_to_position(point_base.point.x-2.13, point_base.point.y, 3-z_depth+0.03)
                    self.control_gripper(True)
                    rospy.sleep(5)  # 等待一段时间，模拟抓取操作
                    self.move_arm_to_position(1, -1, 1)
                    self.control_gripper(False)
                    rospy.signal_shutdown("Task completed, shutting down node")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('yolo_to_robot')
    yolo_to_robot = YoloToRobot()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
