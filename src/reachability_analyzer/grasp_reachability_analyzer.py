import message_utils
import rospy

import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import moveit_commander
import tf
import trajectory_msgs

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

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

        rospy.wait_for_service('compute_ik')
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def _send_pick_request(self, pickup_goal):

        self.pick_plan_client.send_goal(pickup_goal)

        received_result = self.pick_plan_client.wait_for_result()

        if received_result:
            result = self.pick_plan_client.get_result()
            rospy.loginfo("result: " + str(result))
            success = result.error_code.val == result.error_code.SUCCESS
        else:
            # result = None
            # success = False
            rospy.logerr("unable to communicate with movegroup")
            rospy.signal_shutdown("unable to communicate with movegroup")

        rospy.loginfo("success of pick_plan_client:" + str(success))

        return success, result

    def query_moveit_for_reachability(self, graspit_grasp_msg, object_name):
        """
        type: graspit_grasp_msg: graspit_interface.msg.Grasp
        """

        moveit_grasp_msg = message_utils.graspit_interface_to_moveit_grasp(
            graspit_grasp_msg, object_name)

        # pickup_goal = message_utils.build_pickup_goal(
        #     moveit_grasp_msg=moveit_grasp_msg,
        #     object_name=object_name,
        #     allowed_planning_time=self.allowed_planning_time,
        #     planner_id=self.planner_id,
        #     planning_group=self.move_group)

        # success = False
        # result = None
        # try:
        #     self.pick_plan_client.wait_for_server(rospy.Duration(3))

        #     rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))
        #     rospy.loginfo("pickup_goal: " + str(pickup_goal))

        #     success, result = self._send_pick_request(pickup_goal)

        # except Exception as e:
        #     rospy.logerr(
        #         "failed to reach pick action server with err: %s" % e.message)

        # # This section uses direct moveit_commander plan to pose (withough pick_place interface).
        # target_pose = moveit_grasp_msg.grasp_pose
        # rospy.loginfo(target_pose)
        # self.move_group.set_pose_target(target_pose, 'wrist_roll_link')
        # self.move_group.set_planner_id(self.planner_id)
        # plan = self.move_group.plan()
        # success = len(plan.joint_trajectory.points) > 0
        # result = plan


        service_request = PositionIKRequest()
        service_request.group_name = self.move_group.get_name()
        service_request.ik_link_name = self.move_group.get_end_effector_link()
        service_request.pose_stamped = moveit_grasp_msg.grasp_pose
        service_request.timeout.secs= self.allowed_planning_time
        service_request.avoid_collisions = True

        try:
            resp = self.compute_ik(ik_request = service_request)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        success = resp.error_code.val == 1
        result = resp

        return success, result
