#!/usr/bin/env python3

import rospy
import tf
import moveit_commander
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from gazebo_msgs.msg import ModelStates
import sys

class SyncGazeboToMoveIt:
    def __init__(self):
        rospy.init_node('sync_gazebo_to_moveit')
        self.scene = PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.listener = tf.TransformListener()

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.models = {}
        # rospy.loginfo("SyncGazeboToMoveIt node initialized.")

    def gazebo_callback(self, data):
        for i, name in enumerate(data.name):
            self.models[name] = data.pose[i]
            # rospy.loginfo("Model states updated.")

    def update_planning_scene(self):
        rospy.sleep(2)  # 等待初始化

        # 添加桌子
        if 'table' in self.models:
            table_pose = geometry_msgs.msg.PoseStamped()
            table_pose.header.frame_id = self.robot.get_planning_frame()
            table_pose.pose = self.models['table']
            self.scene.add_box("table", table_pose, size=(0.5, 0.5, 0.02))
            # rospy.loginfo("Table added to planning scene.")

        # 添加方形物块
        if 'box_block' in self.models:
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = self.robot.get_planning_frame()
            box_pose.pose = self.models['box_block']
            self.scene.add_box("box_block", box_pose, size=(0.1, 0.1, 0.02))
            # rospy.loginfo("Box block added to planning scene.")

        if 'cylinder_block' in self.models:
            cylinder_pose = geometry_msgs.msg.PoseStamped()
            cylinder_pose.header.frame_id = self.robot.get_planning_frame()
            cylinder_pose.pose = self.models['cylinder_block']
            radius = 0.05  # 圆柱体的半径
            height = 0.02  # 圆柱体的高度
            self.scene.add_cylinder("cylinder_block", cylinder_pose, height=height, radius=radius)

        if 'blue_box' in self.models:
            base_pose = self.models['blue_box']

            # 添加底部
            base_link_pose = geometry_msgs.msg.PoseStamped()
            base_link_pose.header.frame_id = self.robot.get_planning_frame()
            base_link_pose.pose.position.x = base_pose.position.x
            base_link_pose.pose.position.y = base_pose.position.y
            base_link_pose.pose.position.z = base_pose.position.z + 0.025  # 底部厚度的一半
            base_link_pose.pose.orientation = base_pose.orientation
            base_size = (0.5, 0.5, 0.05)
            self.scene.add_box("base_link", base_link_pose, size=base_size)

            # 添加前面
            front_link_pose = geometry_msgs.msg.PoseStamped()
            front_link_pose.header.frame_id = self.robot.get_planning_frame()
            front_link_pose.pose.position.x = base_pose.position.x
            front_link_pose.pose.position.y = base_pose.position.y - 0.225
            front_link_pose.pose.position.z = base_pose.position.z + 0.25 + 0.025  # 高度的一半 + 底部厚度的一半
            front_link_pose.pose.orientation = base_pose.orientation
            front_size = (0.5, 0.05, 0.45)
            self.scene.add_box("front_link", front_link_pose, size=front_size)

            # 添加后面
            back_link_pose = geometry_msgs.msg.PoseStamped()
            back_link_pose.header.frame_id = self.robot.get_planning_frame()
            back_link_pose.pose.position.x = base_pose.position.x
            back_link_pose.pose.position.y = base_pose.position.y + 0.225
            back_link_pose.pose.position.z = base_pose.position.z + 0.25 + 0.025  # 高度的一半 + 底部厚度的一半
            back_link_pose.pose.orientation = base_pose.orientation
            back_size = (0.5, 0.05, 0.45)
            self.scene.add_box("back_link", back_link_pose, size=back_size)

            # 添加左面
            left_link_pose = geometry_msgs.msg.PoseStamped()
            left_link_pose.header.frame_id = self.robot.get_planning_frame()
            left_link_pose.pose.position.x = base_pose.position.x - 0.225
            left_link_pose.pose.position.y = base_pose.position.y
            left_link_pose.pose.position.z = base_pose.position.z + 0.25 + 0.025  # 高度的一半 + 底部厚度的一半
            left_link_pose.pose.orientation = base_pose.orientation
            left_size = (0.05, 0.5, 0.45)
            self.scene.add_box("left_link", left_link_pose, size=left_size)

            # 添加右面
            right_link_pose = geometry_msgs.msg.PoseStamped()
            right_link_pose.header.frame_id = self.robot.get_planning_frame()
            right_link_pose.pose.position.x = base_pose.position.x + 0.225
            right_link_pose.pose.position.y = base_pose.position.y
            right_link_pose.pose.position.z = base_pose.position.z + 0.25 + 0.025  # 高度的一半 + 底部厚度的一半
            right_link_pose.pose.orientation = base_pose.orientation
            right_size = (0.05, 0.5, 0.45)
            self.scene.add_box("right_link", right_link_pose, size=right_size)
        rospy.sleep(1)  # 等待场景更新

    def run(self):
        self.update_planning_scene()
        rospy.signal_shutdown("Planning scene initialization complete")

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        sync = SyncGazeboToMoveIt()
        sync.run()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass
