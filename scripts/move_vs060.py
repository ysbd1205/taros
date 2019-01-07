#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf


class MoveVS060:
    def __init__(self):
        self.group_name = "arm"
        self.base_name = "base_link"
        self.ee_name = "bucket_link"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.listener = tf.TransformListener()

    def get_current_pose(self):
        """
        For getting current end effector pose.
        :Return: Tuple of (position, orientation)

            Usage:
                p, o = get_current_pose()
        """
        pose = self.group.get_current_pose().pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return position, orientation

    def get_current_joints(self):
        """
        For getting current joints.
        :Return: List of joint angles

            Usage:
                j = get_current_joints()
        """
        joint_states = self.group.get_current_joint_values()
        return joint_states

    def move_to_pose(self, position, orientation):
        """
        This is move function based on end effector pose.
        Here, end effector is 'bucket_link'
        :Input: List of ee position => [x, y, z]
        :Input: List of ee orientation by quaternion => [x, y, z, w]

            Usage:
                move_to_pose([0, 0, 0.1], [0.1, 0.2, 0.1, 0.4)
        """
        assert len(position) == 3, "Number of position elements is {}. It's supposed to 3!".format(len(position))
        assert len(orientation) == 4, "Number of orientation elements is {}. It's supposed to 4!".format(len(orientation))

        pose_goal = Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.clear_pose_targets()

    def move_to_joint(self, joint_goal):
        """
        This is move function based on each joint position.
        :Input: List of joint position => [joint_j1, joint_j2, joint_j3, joint_j4, joint_j5, joint_j6]

            Usage:
                move_to_joint_position([0, 0, 0.1, 0.2, 0.5, 0.0])
        """
        assert type(joint_goal) == list, "Input must be List!"
        assert len(joint_goal) == 6, "Number of elements is {}. It's supposed to 6!".format(len(joint_goal))

        self.group.go(joint_goal, wait=True)

    def cartesian_move(self, position):
        """
        This is move function via cartesian path.
        Cartesian path is movement along with the line between two points like tool-movement of RC8.
        One of two points is current end effector pose.
        :Input: List of ee position => [x, y, z]

            Usage:
                cartesian_move([0.1, 0.0, 0.0]) # then 0.1[m] move along with x-axis
                cartesian_move([0.0, 0.1, 0.0]) # then 0.1[m] move along with y-axis
                cartesian_move([0.0, 0.0, 0.1]) # then 0.1[m] move along with z-axis
        """
        assert len(position) == 3, "Number of position elements is {}. It's supposed to 3!".format(len(position))

        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.x += position[0]
        wpose.position.y += position[1]
        wpose.position.z += position[2]
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    def show_info(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", group_names

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""
