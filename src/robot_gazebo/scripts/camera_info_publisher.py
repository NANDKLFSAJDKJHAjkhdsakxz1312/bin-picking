#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo
import yaml

def load_camera_info(yaml_file):
    with open(yaml_file, "r") as file_handle:
        calib_data = yaml.load(file_handle, Loader=yaml.SafeLoader)
        camera_info_msg = CameraInfo()

        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data.get("distortion_model", "plumb_bob")
        
        return camera_info_msg

def publish_camera_info(yaml_file):
    pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    rospy.init_node('camera_info_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    camera_info_msg = load_camera_info(yaml_file)

    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.header.frame_id = 'depth_camera_link'
        pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        yaml_file_path = "/home/jiangnan/labor_robotik/src/ost.yaml"  # 修改为你的ost.yaml文件路径
        publish_camera_info(yaml_file_path)
    except rospy.ROSInterruptException:
        pass
