#!/usr/bin/env python

import sys

import rospy
import moveit_commander
import actionlib

from grasp_reachability_analyzer import GraspReachabilityAnalyzer
import reachability_analyzer.msg


def param(ns, name):
    full_param_name = rospy.search_param('/' + ns + '/' + name)
    if not full_param_name:
        raise Exception("Parameter Not Found: " + str(name))
    return rospy.get_param(full_param_name)


class GraspAnalyzerNode(object):
    _result = reachability_analyzer.msg.CheckGraspReachabilityResult()

    def __init__(self):

        ns = 'reachability_analyzer'
        rospy.init_node(ns)

        analyze_grasp_topic = param(ns, "analyze_grasp_topic")
        arm_move_group_name = param(ns, 'arm_move_group_name')
        hand_move_group_name = param(ns, 'hand_move_group_name')
        planner_id = arm_move_group_name + param(ns, 'planner_config_name')
        allowed_planning_time = param(ns, 'allowed_planning_time')

        moveit_commander.roscpp_initialize(sys.argv)

        arm_group = moveit_commander.MoveGroupCommander(arm_move_group_name)
        hand_group = moveit_commander.MoveGroupCommander(hand_move_group_name)

        self.grasp_reachability_analyzer = GraspReachabilityAnalyzer(
            ns, arm_group, hand_group, planner_id, allowed_planning_time)

        self._analyze_grasp_as = actionlib.SimpleActionServer(
            analyze_grasp_topic,
            reachability_analyzer.msg.CheckGraspReachabilityAction,
            execute_cb=self.analyze_grasp_reachability_cb,
            auto_start=False)

        self._analyze_grasp_as.start()

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def analyze_grasp_reachability_cb(self, goal):
        """
        @return: Whether the grasp is expected to succeed
        @rtype: bool
        """
        success, result = self.grasp_reachability_analyzer.query_moveit_for_reachability(
            goal.grasp, goal.grasp_frame_id)

        self._result.isReachable = success

        rospy.loginfo(self.__class__.__name__ +
                      " finished analyze grasp request: " + str(self._result))
        self._analyze_grasp_as.set_succeeded(self._result)
        return self._result


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
