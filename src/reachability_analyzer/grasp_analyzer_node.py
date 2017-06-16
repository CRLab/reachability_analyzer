#!/usr/bin/env python

import rospy
import roslib

import graspit_msgs.msg

from grasp_reachability_analyzer import GraspReachabilityAnalyzer

import sys
import moveit_commander
import actionlib

import graspit_interface.msg


class GraspAnalyzerNode(object):

    def __init__(self):
        rospy.init_node('grasp_analyzer_node')
        analyze_grasp_topic = rospy.get_param('analyze_grasp_topic')
        move_group_name = rospy.get_param('move_group_name')
        # grasp_approach_tran_frame = rospy.get_param('grasp_approach_tran_frame')
        planner_id = move_group_name + rospy.get_param('reachability_analyzer/planner_config_name')
        allowed_planning_time = rospy.get_param('reachability_analyzer/allowed_planning_time')

        moveit_commander.roscpp_initialize(sys.argv)

        group = moveit_commander.MoveGroupCommander(move_group_name)

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(group, planner_id, allowed_planning_time)

        self._analyze_grasp_as = actionlib.SimpleActionServer(analyze_grasp_topic,
                                                              graspit_interface.msg.CheckGraspReachabilityAction,
                                                              execute_cb=self.analyze_grasp_reachability_cb,
                                                              auto_start=False)
        self._analyze_grasp_as.start()

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def analyze_grasp_reachability_cb(self, goal):
        """
        @return: Whether the grasp is expected to succeed
        @rtype: bool
        """
        success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(goal.grasp)
        _result = graspit_msgs.msg.CheckGraspReachabilityResult()
        _result.isPossible = success
        _result.grasp_id = goal.grasp.grasp_id
        rospy.loginfo(self.__class__.__name__ + " finished analyze grasp request: " + str(_result))
        self._analyze_grasp_as.set_succeeded(_result)
        return _result


def main():
    try:
        grasp_analyzer_node = GraspAnalyzerNode()
        loop = rospy.Rate(10)

        while not rospy.is_shutdown():
            loop.sleep()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown()


if __name__ == '__main__':
    main()
