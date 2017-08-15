import message_utils
import rospy

import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import moveit_commander
import tf
import trajectory_msgs


class GraspReachabilityAnalyzer():
    def __init__(self, move_group, planner_id, allowed_planning_time):
        """
        :type move_group: moveit_commander.MoveGroupCommander
        :type planner_id: str
        :type allowed_planning_time: int
        """
        self.pick_plan_client = actionlib.SimpleActionClient(
            '/pickup', moveit_msgs.msg.PickupAction)
        self.move_group = move_group
        self.planner_id = planner_id
        # this is how much time moveit thinks it has
        self.allowed_planning_time = allowed_planning_time
        # this is how long we wait for a response from moveit
        self.planner_timeout = allowed_planning_time + 1

    def _send_pick_request(self, pickup_goal):

        self.pick_plan_client.send_goal(pickup_goal)

        received_result = self.pick_plan_client.wait_for_result(
            rospy.Duration(self.planner_timeout))

        if received_result:
            result = self.pick_plan_client.get_result()
            rospy.loginfo("result: " + str(result))
            success = result.error_code.val == result.error_code.SUCCESS
        else:
            result = None
            success = False

        rospy.loginfo("success of pick_plan_client:" + str(success))

        return success, result

    def query_moveit_for_reachability(self, graspit_grasp_msg):
        """
        type: graspit_grasp_msg: graspit_interface.msg.Grasp
        """

        moveit_grasp_msg = message_utils.graspit_interface_to_moveit_grasp(
            graspit_grasp_msg)

        pickup_goal = message_utils.build_pickup_goal(
            moveit_grasp_msg=moveit_grasp_msg,
            object_name=graspit_grasp_msg.object_name,
            allowed_planning_time=self.allowed_planning_time,
            planner_id=self.planner_id,
            planning_group=self.move_group)

        success = False
        result = None
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))

            rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))
            rospy.loginfo("pickup_goal: " + str(pickup_goal))

            success, result = self._send_pick_request(pickup_goal)

        except Exception as e:
            rospy.logerr(
                "failed to reach pick action server with err: %s" % e.message)

        return success, result
