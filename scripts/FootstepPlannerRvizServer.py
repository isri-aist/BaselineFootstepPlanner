#!/usr/bin/env python3

import copy
import numpy as np
from rclpy.node import Node
import rclpy
import ast
# from tf import transformations
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from baseline_footstep_planner.msg import FootstepSequence2DStamped

def toPoseMsg(pose_2d, z=0.0):
    """"Get geometry_msgs::Pose message.

    Args:
        pose_2d (list): 2D pose [X position, Y position, Yaw orientation].
        z (float): Z position.
    """
    pose_msg = Pose()
    pos = pose_msg.position
    pos.x, pos.y, pos.z = pose_2d[0], pose_2d[1], z
    ori = pose_msg.orientation
    ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(0.0, 0.0, pose_2d[2])
    return pose_msg

class FootstepPlannerRvizServer(Node):
    """"Node to manage ROS topics for Rviz."""
    def __init__(self):
        super().__init__('footstep_planner_rviz_server')
        # get param
        self.nominal_foot_separation = self.declare_parameter("nominal_foot_separation", 0.3).value # [m]
        self.foot_size = self.declare_parameter("foot_size", [0.15, 0.075]).value # [m]
        self.setupRectObstList()
        self.timer = self.create_timer(1.0, self.publishObstMarkerArray)
        # setup marker
        self.setupInteractiveMarker()

        # setup subscriber
        self.footstep_seq_sub = self.create_subscription(
            FootstepSequence2DStamped, "footstep_sequence", self.footstepSeqCallback, 1)

        # setup publisher
        self.start_pose_pub = self.create_publisher(Pose2D, "start_pose", 1)
        self.goal_pose_pub = self.create_publisher(Pose2D, "goal_pose", 1)
        self.footstep_seq_marker_pub = self.create_publisher(MarkerArray, "footstep_sequence_marker", 1)
        self.footstep_obsts_marker_pub = self.create_publisher(MarkerArray, "footstep_obstacles_marker", 1)

    def setupRectObstList(self):
        self.rect_obst_list = []
        rect_obstacle_string = self.declare_parameter("rect_obstacle_list", "").value
        rect_obstacle_list = ast.literal_eval(rect_obstacle_string)

        for rect_obst_param in rect_obstacle_list:
            self.rect_obst_list.append(np.array(rect_obst_param))

    def setupInteractiveMarker(self):
        # make foot markers
        self.both_foot_markers = self._makeBothFootMarkers()

        # make server
        self.im_server = InteractiveMarkerServer(self, "footstep_planner")

        # add start
        self.im_server.insert(
            self._makeInteractiveMarkerSe2(
                name="start",
                pose_2d=[0.0, 0.0, 0.0],
                markers=copy.copy(self.both_foot_markers)),
            feedback_callback = self.interactivemarkerFeedback)

        # add goal
        self.im_server.insert(
            self._makeInteractiveMarkerSe2(
                name="goal",
                pose_2d=[2.0, 1.5, 0.0],
                markers=copy.copy(self.both_foot_markers)),
            feedback_callback = self.interactivemarkerFeedback)

        # apply to server
        self.im_server.applyChanges()

    def _makeBothFootMarkers(self):
        both_foot_markers = []
        for i in [-1, 1]:
            footstep_marker = Marker()
            footstep_marker.type = Marker.CUBE
            if i == -1:
                footstep_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            else:
                footstep_marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
            footstep_marker.scale.x = self.foot_size[0]
            footstep_marker.scale.y = self.foot_size[1]
            footstep_marker.scale.z = 0.01
            footstep_marker.pose = toPoseMsg([0.0, i * 0.5 * self.nominal_foot_separation, 0.0])
            both_foot_markers.append(footstep_marker)
        return both_foot_markers

    def _makeInteractiveMarkerSe2(self, name, pose_2d, markers=None):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = name
        int_marker.pose = toPoseMsg(pose_2d)

        # marker control
        if markers is not None:
            for i, marker in enumerate(markers):
                control = InteractiveMarkerControl()
                control.name = "marker{}".format(i)
                control.always_visible = True
                control.interaction_mode = InteractiveMarkerControl.NONE
                control.markers.append(marker)
                int_marker.controls.append(control)

        # rotate_z control
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(0, np.pi/2, 0)
        int_marker.controls.append(control)

        # move_x control
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(0, 0, 0)
        int_marker.controls.append(control)

        # move_y control
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = quaternion_from_euler(0, 0, np.pi/2)
        int_marker.controls.append(control)

        return int_marker

    def interactivemarkerFeedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_UP:
            # set message
            pose_msg = Pose2D()
            pose_msg.x = feedback.pose.position.x
            pose_msg.y = feedback.pose.position.y
            quat = feedback.pose.orientation
            euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            pose_msg.theta = euler[2]

            # publish start/goal
            if feedback.marker_name == "start":
                self.start_pose_pub.publish(pose_msg)
            elif feedback.marker_name == "goal":
                self.goal_pose_pub.publish(pose_msg)

    def publishObstMarkerArray(self):
        stamp_now = self.get_clock().now().to_msg()

        # instantiate marker array
        obsts_marker_arr = MarkerArray()

        # delete marker
        del_marker = Marker(id=0, action=Marker.DELETEALL)
        del_marker.header.stamp = stamp_now
        del_marker.header.frame_id = "world"
        obsts_marker_arr.markers.append(del_marker)

        # cube marker
        for rect_obst in self.rect_obst_list:
            rect_center = rect_obst[0:2]
            rect_half_length = rect_obst[2:4]

            obst_marker = Marker()
            obst_marker.header.stamp = stamp_now
            obst_marker.header.frame_id = "world"
            obst_marker.id = len(obsts_marker_arr.markers)
            obst_marker.type = Marker.CUBE
            obst_marker.color = ColorRGBA(r=0.2, g=0.2, b=0.8, a=0.5)
            obst_marker.scale.x = 2 * rect_half_length[0]
            obst_marker.scale.y = 2 * rect_half_length[1]
            obst_marker.scale.z = 0.01
            obst_marker.pose = toPoseMsg([rect_center[0], rect_center[1], 0], z=-0.01)
            obsts_marker_arr.markers.append(obst_marker)

        # publish marker array
        self.footstep_obsts_marker_pub.publish(obsts_marker_arr)

    def footstepSeqCallback(self, footstep_seq_msg):
        # instantiate marker array
        seq_marker_arr = MarkerArray()

        # delete marker
        del_marker = Marker(id=0, action=Marker.DELETEALL)
        del_marker.header.stamp = footstep_seq_msg.header.stamp
        del_marker.header.frame_id = "world"
        seq_marker_arr.markers.append(del_marker)

        # cube marker
        for footstep_msg in footstep_seq_msg.sequence.footsteps:
            footstep_marker = Marker()
            footstep_marker.header.stamp = footstep_seq_msg.header.stamp
            footstep_marker.header.frame_id = "world"
            footstep_marker.id = len(seq_marker_arr.markers)
            footstep_marker.type = Marker.CUBE
            if footstep_msg.foot_lr == footstep_msg.LEFT:
                footstep_marker.color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.8)
            else:
                footstep_marker.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)
            footstep_marker.scale.x = self.foot_size[0]
            footstep_marker.scale.y = self.foot_size[1]
            footstep_marker.scale.z = 0.01
            footstep_marker.pose = toPoseMsg(
                [footstep_msg.foot_pose.x, footstep_msg.foot_pose.y, footstep_msg.foot_pose.theta], z=-0.01)
            seq_marker_arr.markers.append(footstep_marker)

        # lines marker
        lines_marker = Marker()
        lines_marker.header.stamp = footstep_seq_msg.header.stamp
        lines_marker.header.frame_id = "world"
        lines_marker.id = len(seq_marker_arr.markers)
        # LINE_STRIP has a problem in visualization (line thickness is not constant depending on the viewpoint)
        lines_marker.color = ColorRGBA(r=0.2, g=0.2, b=0.2, a=0.5)
        lines_marker.scale.x = 0.01 # line width
        lines_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        for i in range(len(footstep_seq_msg.sequence.footsteps) - 1):
            current_pose = footstep_seq_msg.sequence.footsteps[i].foot_pose
            next_pose = footstep_seq_msg.sequence.footsteps[i + 1].foot_pose
            lines_marker.points.append(Point(x=current_pose.x, y=current_pose.y, z=0.0))
            lines_marker.points.append(Point(x=next_pose.x, y=next_pose.y, z=0.0))
        seq_marker_arr.markers.append(lines_marker)

        # publish marker array
        self.footstep_seq_marker_pub.publish(seq_marker_arr)

    def spin(self):
        rclpy.spin(self)


if __name__ == "__main__":
    rclpy.init()
    rviz_server = FootstepPlannerRvizServer()
    rviz_server.spin()
    rclpy.shutdown()