
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
        """
        self.pick_plan_client = actionlib.SimpleActionClient('/pickup', moveit_msgs.msg.PickupAction)
        self.move_group = move_group
        self.planner_id = planner_id
        self.allowed_planning_time = allowed_planning_time # this is how much time moveit thinks it has
        self.planner_timeout = allowed_planning_time + 1 #this is how long we wait for a response from moveit

    def send_pick_request(self, pickup_goal):

        success = False
        received_result = False
        self.pick_plan_client.send_goal(pickup_goal)

        received_result = self.pick_plan_client.wait_for_result(rospy.Duration(self.planner_timeout))

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

        moveit_grasp_msg = message_utils.graspit_grasp_to_moveit_grasp(graspit_grasp_msg)

        
        pickup_goal = message_utils.build_pickup_goal(moveit_grasp_msg=moveit_grasp_msg,
                                                      object_name=graspit_grasp_msg.object_name,
                                                      allowed_planning_time = self.allowed_planning_time,
                                                      planner_id=self.planner_id,
                                                      planning_group=self.move_group)

        received_result = False
        success  = False
        try:
            self.pick_plan_client.wait_for_server(rospy.Duration(3))

            rospy.loginfo("moveit_grasp_msg: " + str(moveit_grasp_msg))
            rospy.loginfo("pickup_goal: " + str(pickup_goal))

            # import IPython
            # IPython.embed()

            success, result = self.send_pick_request(pickup_goal)

        except Exception as e:
            rospy.logerr("failed to reach pick action server with err: %s" % e.message)


        return success, result