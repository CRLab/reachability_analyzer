
import moveit_msgs.msg
import trajectory_msgs.msg
import rospy


def graspit_interface_to_moveit_grasp(graspit_interface_grasp_msg, grasp_frame_id):
    """
    :param graspit_interface_grasp_msg: A graspit_interface grasp message
    :type graspit_interface_grasp_msg: graspit_interface.msg.Grasp
    :returns a moveit message built from the graspit grasp
    :rtype: moveit_msgs.msg.Grasp
    """
    moveit_grasp = moveit_msgs.msg.Grasp()
    # # This message contains a description of a grasp that would be used
    # # with a particular end-effector to grasp an object, including how to
    # # approach it, grip it, etc.  This message does not contain any
    # # information about a "grasp point" (a position ON the object).
    # # Whatever generates this message should have already combined
    # # information about grasp points with information about the geometry
    # # of the end-effector to compute the grasp_pose in this message.
    #
    # # A name for this grasp
    # string id
    #
    moveit_grasp.id = str(graspit_interface_grasp_msg.__hash__())

    # # The internal posture of the hand for the pre-grasp
    # # only positions are used
    #
    # trajectory_msgs/JointTrajectory pre_grasp_posture
    #
    pre_grasp_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    pre_grasp_goal_point.effort = rospy.get_param("pre_grasp_goal_point.effort")
    pre_grasp_goal_point.positions = rospy.get_param("pre_grasp_goal_point.positions")
    pre_grasp_goal_point.time_from_start.secs = rospy.get_param("pre_grasp_goal_point.time_from_start.secs")

    moveit_grasp.pre_grasp_posture.points.append(pre_grasp_goal_point)
    moveit_grasp.pre_grasp_posture.joint_names = rospy.get_param("pre_grasp_joint_names")

    # # The internal posture of the hand for the grasp
    # # positions and efforts are used
    #
    # trajectory_msgs/JointTrajectory grasp_posture
    #
    grasp_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    grasp_goal_point.effort = rospy.get_param("grasp_goal_point.effort")
    grasp_goal_point.positions = graspit_interface_grasp_msg.dofs
    grasp_goal_point.time_from_start.secs = rospy.get_param("grasp_goal_point.time_from_start.secs")

    moveit_grasp.grasp_posture.points.append(grasp_goal_point)
    moveit_grasp.grasp_posture.joint_names = rospy.get_param("moveit_grasp.grasp_posture.joint_names")

    # # The position of the end-effector for the grasp.  This is the pose of
    # # the "parent_link" of the end-effector, not actually the pose of any
    # # link *in* the end-effector.  Typically this would be the pose of the
    # # most distal wrist link before the hand (end-effector) links began.
    #
    # geometry_msgs/PoseStamped grasp_pose
    #
    moveit_grasp.grasp_pose.pose = graspit_interface_grasp_msg.pose
    moveit_grasp.grasp_pose.header.frame_id = grasp_frame_id

    # # The estimated probability of success for this grasp, or some other
    # # measure of how "good" it is.
    #
    # float64 grasp_quality
    #
    moveit_grasp.grasp_quality = graspit_interface_grasp_msg.epsilon_quality

    # # The approach direction to take before picking an object
    #
    # GripperTranslation pre_grasp_approach
    #
    moveit_grasp.pre_grasp_approach.min_distance = rospy.get_param("moveit_grasp.pre_grasp_approach.min_distance")
    moveit_grasp.pre_grasp_approach.desired_distance = rospy.get_param("moveit_grasp.pre_grasp_approach.desired_distance")
    moveit_grasp.pre_grasp_approach.direction.header.frame_id = rospy.get_param("moveit_grasp.pre_grasp_approach.direction.header.frame_id")
    moveit_grasp.pre_grasp_approach.direction.vector = graspit_interface_grasp_msg.approach_direction.vector


    # # The retreat direction to take after a grasp has been completed (object is attached)
    #
    # GripperTranslation post_grasp_retreat
    #
    moveit_grasp.post_grasp_retreat.min_distance = rospy.get_param("moveit_grasp.post_grasp_retreat.min_distance")
    moveit_grasp.post_grasp_retreat.desired_distance = rospy.get_param("moveit_grasp.post_grasp_retreat.desired_distance")
    moveit_grasp.post_grasp_retreat.direction.header.frame_id = rospy.get_param("moveit_grasp.post_grasp_retreat.direction.header.frame_id")
    moveit_grasp.post_grasp_retreat.direction.vector.x = rospy.get_param("moveit_grasp.post_grasp_retreat.direction.vector.x")
    moveit_grasp.post_grasp_retreat.direction.vector.y = rospy.get_param("moveit_grasp.post_grasp_retreat.direction.vector.y")
    moveit_grasp.post_grasp_retreat.direction.vector.z = rospy.get_param("moveit_grasp.post_grasp_retreat.direction.vector.z")

    # # The retreat motion to perform when releasing the object; this information
    # # is not necessary for the grasp itself, but when releasing the object,
    # # the information will be necessary. The grasp used to perform a pickup
    # # is returned as part of the result, so this information is available for
    # # later use.
    #
    # GripperTranslation post_place_retreat
    #

    # # the maximum contact force to use while grasping (<=0 to disable)
    #
    # float32 max_contact_force
    #
    moveit_grasp.max_contact_force = rospy.get_param("moveit_grasp.max_contact_force")

    # # an optional list of obstacles that we have semantic information about
    # # and that can be touched/pushed/moved in the course of grasping
    #
    # string[] allowed_touch_objects
    #
    moveit_grasp.allowed_touch_objects = []

    return moveit_grasp


def build_pickup_goal(moveit_grasp_msg, object_name, allowed_planning_time,planner_id, planning_group):
    """
    :type planning_group: moveit_commander.MoveGroupCommander
    """

    pickup_goal = moveit_msgs.msg.PickupGoal()

    # # An action for picking up an object
    #
    # # The name of the object to pick up (as known in the planning scene)
    #
    # string target_name
    #
    pickup_goal.target_name = object_name

    # # which group should be used to plan for pickup
    #
    # string group_name
    #
    pickup_goal.group_name = planning_group.get_name()

    # # which end-effector to be used for pickup (ideally descending from the group above)
    #
    # string end_effector
    #
    pickup_goal.end_effector = rospy.get_param('end_effector_name')

    # # a list of possible grasps to be used. At least one grasp must be filled in
    #
    # Grasp[] possible_grasps
    #
    pickup_goal.possible_grasps = [moveit_grasp_msg]

    # # the name that the support surface (e.g. table) has in the collision map
    # # can be left empty if no name is available
    #
    # string support_surface_name
    #
    # pickup_goal.support_surface_name = "table"

    # # whether collisions between the gripper and the support surface should be acceptable
    # # during move from pre-grasp to grasp and during lift. Collisions when moving to the
    # # pre-grasp location are still not allowed even if this is set to true.
    #
    # bool allow_gripper_support_collision
    #
    pickup_goal.allow_gripper_support_collision = False

    # # The names of the links the object to be attached is allowed to touch;
    # # If this is left empty, it defaults to the links in the used end-effector
    #
    # string[] attached_object_touch_links
    #
    # pickup_goal.attached_object_touch_links = []

    # # Optionally notify the pick action that it should approach the object further,
    # # as much as possible (this minimizing the distance to the object before the grasp)
    # # along the approach direction; Note: this option changes the grasping poses
    # # supplied in possible_grasps[] such that they are closer to the object when possible.
    #
    # bool minimize_object_distance
    #
    pickup_goal.minimize_object_distance = False

    # # Optional constraints to be imposed on every point in the motion plan
    # Constraints path_constraints
    #
    # # The name of the motion planner to use. If no name is specified,
    # # a default motion planner will be used
    #
    # string planner_id
    #
    pickup_goal.planner_id = planner_id

    # # an optional list of obstacles that we have semantic information about
    # # and that can be touched/pushed/moved in the course of grasping;
    # # CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.
    #
    # string[] allowed_touch_objects
    #
    pickup_goal.allowed_touch_objects = ['all']

    # # The maximum amount of time the motion planner is allowed to plan for
    #
    # float64 allowed_planning_time
    #
    pickup_goal.allowed_planning_time = allowed_planning_time

    # # Planning options
    #
    # PlanningOptions planning_options
    #
    pickup_goal.planning_options.plan_only = True
    pickup_goal.planning_options.replan = False
    pickup_goal.planning_options.look_around = False
    pickup_goal.planning_options.replan_delay = 10.0

    ## Constraints
    
    # joint_tolerance = rospy.get_param('joint_path_tolerance',math.pi)
    # rospy.loginfo('joint tolerance constraint in planner %f'%(joint_tolerance))
    # joint_names = planning_group.get_joints()
    # joint_values = planning_group.get_current_joint_values()
    # joint_constraints = [moveit_msgs.msg.JointConstraint(joint_name=name, position=val, tolerance_above=joint_tolerance,
    #                                                      tolerance_below=joint_tolerance, weight=1)
    #                      for name, val in zip(joint_names, joint_values)]

    #pickup_goal.path_constraints.joint_constraints = joint_constraints

    return pickup_goal
