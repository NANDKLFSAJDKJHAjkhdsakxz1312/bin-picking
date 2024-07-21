#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty

def control_gripper(state):
    service_name = '/cylinder_block/vacuum_gripper/on' if state else '/cylinder_block/vacuum_gripper/off'
    rospy.wait_for_service(service_name)
    try:
        service_proxy = rospy.ServiceProxy(service_name, Empty)
        resp = service_proxy()
        rospy.loginfo("Service %s called successfully", service_name)
        return resp
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node("gripper_controller", anonymous=False)
    try:
        control_gripper(True)  # 调用服务以开启吸盘
        rospy.sleep(1000)  # 等待一段时间，模拟吸盘吸附操作

        control_gripper(False)  # 调用服务以关闭吸盘
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
