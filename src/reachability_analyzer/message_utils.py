
import tf_conversions.posemath as pm

import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import moveit_commander
import tf
import rospy
import tf_conversions
import numpy as np
import ipdb
import math



def barrett_positions_from_graspit_positions(positions):
    """
    :param positions: Positions of the barrett hand in graspit's convention
    :type positions: numpy.array
    :returns a pair containing the joint names array and the joint positions
    :rtype: (list[string],numpy.array)
    """
    names = ['/finger_1/prox_joint', '/finger_1/med_joint', '/finger_1/dist_joint',
             '/finger_2/prox_joint', '/finger_2/med_joint', '/finger_2/dist_joint',
             '/finger_3/med_joint', '/finger_3/dist_joint']
    prefix_str = 'wam/bhand'
    joint_names = [prefix_str + name for name in names]
    joint_positions = np.zeros([len(names)])
    joint_positions[0] = positions[0]
    joint_positions[1] = positions[1]
    joint_positions[2] = positions[1]/3.0
    joint_positions[3] = positions[0]
    joint_positions[4] = positions[2]
    joint_positions[5] = positions[2]/3.0
    joint_positions[6] = positions[3]
    joint_positions[7] = positions[3]/3.0

    return joint_names, joint_positions.tolist()


def mico_positions_from_graspit_positions(positions):
    """
    :param positions: Positions of the mico arm in graspit's conventions
    :type positions: numpy.array
    :returns a pair containing the joint names array & the joint positions
    :rtype (list[string],list[float])
    """
    joint_positions = [positions[0], positions[0]]
    joint_names = ['mico_joint_finger_1', 'mico_joint_finger_2']
    return joint_names, joint_positions

def pr2_positions_from_graspit_positions(positions):
    """
    :param positions: Positions of the pr2 arm in graspit's conventions
    :type positions: numpy.array
    :returns a pair containing the joint names array & the joint positions
    :rtype (list[string],list[float])
    """
    joint_names = ['l_gripper_l_finger_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_joint']
    joint_positions = [0.1, 0.1, 0.1, 0.1]
    return joint_names, joint_positions


def fetch_positions_from_graspit_positions(positions):
    """
    :param positions: Positions of the mico arm in graspit's conventions
    :type positions: numpy.array
    :returns a pair containing the joint names array & the joint positions
    :rtype (list[string],list[float])
    """
    joint_positions = [0.1, 0.1]
    joint_names = ['l_gripper_finger_joint', 'r_gripper_finger_joint']
    return joint_names, joint_positions


def graspit_grasp_pose_to_moveit_grasp_pose(move_group_commander, listener, graspit_grasp_msg,
                                            grasp_frame='/approach_tran'):
    """
    :param move_group_commander: A move_group command from which to get the end effector link.
    :type move_group_commander: moveit_commander.MoveGroupCommander
    :param listener: A transformer for looking up the transformation
    :type tf.TransformListener 
    :param graspit_grasp_msg: A graspit grasp message
    :type graspit_grasp_msg: graspit_msgs.msg.Grasp

    """

    try:
        listener.waitForTransform(grasp_frame, move_group_commander.get_end_effector_link(),
                                     rospy.Time(0), timeout=rospy.Duration(1))
        at_to_ee_tran, at_to_ee_rot = listener.lookupTransform(grasp_frame, move_group_commander.get_end_effector_link(),rospy.Time())
    except:
        rospy.logerr("graspit_grasp_pose_to_moveit_grasp_pose::\n " +
                    "Failed to find transform from %s to %s"%(grasp_frame, move_group_commander.get_end_effector_link()))
        ipdb.set_trace()



    graspit_grasp_msg_final_grasp_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(graspit_grasp_msg.final_grasp_pose))
    approach_tran_to_end_effector_tran_matrix = tf.TransformerROS().fromTranslationRotation(at_to_ee_tran, at_to_ee_rot)
    actual_ee_pose_matrix = np.dot( graspit_grasp_msg_final_grasp_tran_matrix, approach_tran_to_end_effector_tran_matrix)
    actual_ee_pose = tf_conversions.toMsg(tf_conversions.fromMatrix(actual_ee_pose_matrix))
    rospy.loginfo("actual_ee_pose: " + str(actual_ee_pose))
    return actual_ee_pose

def graspit_interface_grasp_pose_to_moveit_grasp_pose(move_group_commander, listener, graspit_interface_grasp_msg,
                                            grasp_frame='/approach_tran'):
    """
    :param move_group_commander: A move_group command from which to get the end effector link.
    :type move_group_commander: moveit_commander.MoveGroupCommander
    :param listener: A transformer for looking up the transformation
    :type tf.TransformListener 
    :param graspit_interface_grasp_msg: A graspit grasp message
    :type graspit_interface_grasp_msg: graspit_interface_msgs.msg.Grasp

    """

    try:
        listener.waitForTransform(grasp_frame, move_group_commander.get_end_effector_link(),
                                     rospy.Time(0), timeout=rospy.Duration(1))
        at_to_ee_tran, at_to_ee_rot = listener.lookupTransform(grasp_frame, move_group_commander.get_end_effector_link(),rospy.Time())
    except:
        rospy.logerr("graspit_grasp_pose_to_moveit_grasp_pose::\n " +
                    "Failed to find transform from %s to %s"%(grasp_frame, move_group_commander.get_end_effector_link()))
        ipdb.set_trace()



    graspit_grasp_msg_final_grasp_tran_matrix = tf_conversions.toMatrix(tf_conversions.fromMsg(graspit_interface_grasp_msg.pose))
    approach_tran_to_end_effector_tran_matrix = tf.TransformerROS().fromTranslationRotation(at_to_ee_tran, at_to_ee_rot)
    actual_ee_pose_matrix = np.dot( graspit_grasp_msg_final_grasp_tran_matrix, approach_tran_to_end_effector_tran_matrix)
    actual_ee_pose = tf_conversions.toMsg(tf_conversions.fromMatrix(actual_ee_pose_matrix))
    rospy.loginfo("actual_ee_pose: " + str(actual_ee_pose))
    return actual_ee_pose


def get_approach_dir_in_ee_coords(move_group_commander, listener, approach_dir_stamped):
    """
    :type move_group_commander: moveit_commander.MoveGroupCommander
    :type listener: tf.TransformListener
    :type approach_dir_stamped: geometry_msgs.msg.Vector3Stamped
    :rtype geometry_msgs.msg.Vector3Stamped
    """
    try:        
        listener.waitForTransform(approach_dir_stamped.header.frame_id, move_group_commander.get_end_effector_link(),
                                     rospy.Time(), timeout=rospy.Duration(2))
        listener.lookupTransform(approach_dir_stamped.header.frame_id, move_group_commander.get_end_effector_link(),rospy.Time())
    except:
        rospy.logerr("Failed to find transform from %s to %s"%(approach_dir_stamped.header.frame_id, move_group_commander.get_end_effector_link()))
        ipdb.set_trace()

    approach_dir_transformed = listener.transformVector3(move_group_commander.get_end_effector_link(), approach_dir_stamped)
    return approach_dir_transformed


def graspit_grasp_to_moveit_grasp(graspit_grasp_msg, move_group_commander, listener,  grasp_tran_frame_name='approach_tran'):
    """
    :param graspit_grasp_msg: A graspit grasp message
    :type graspit_grasp_msg: graspit_msgs.msg.Grasp
    :returns a moveit message built from the graspit grasp
    :rtype: moveit_msgs.msg.Grasp
    """
    # 'manipulator' is the name for the root move group in the mico arm
    moveit_positions_from_graspit_positions = {'StaubliArm': barrett_positions_from_graspit_positions,
                                               'manipulator': mico_positions_from_graspit_positions,
                                               'arm': fetch_positions_from_graspit_positions,
                                               'left_arm': pr2_positions_from_graspit_positions}
    move_group_name = rospy.get_param('/move_group_name')
    moveit_positions_from_graspit_positions_fcn = moveit_positions_from_graspit_positions[move_group_name]

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
    moveit_grasp.id = 'graspit_' + str(graspit_grasp_msg.grasp_id)

    # # The internal posture of the hand for the pre-grasp
    # # only positions are used
    #
    # trajectory_msgs/JointTrajectory pre_grasp_posture
    #
    pre_grasp_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    spread_pregrasp_dof = (0, 0, 0, 0)
    pre_grasp_joint_names, pre_grasp_goal_point.positions = moveit_positions_from_graspit_positions_fcn(spread_pregrasp_dof)
    moveit_grasp.pre_grasp_posture.points.append(pre_grasp_goal_point)
    moveit_grasp.pre_grasp_posture.joint_names = pre_grasp_joint_names

    # # The internal posture of the hand for the grasp
    # # positions and efforts are used
    #
    # trajectory_msgs/JointTrajectory grasp_posture
    #
    goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    joint_names, goal_point.positions = moveit_positions_from_graspit_positions_fcn(graspit_grasp_msg.pre_grasp_dof)
    moveit_grasp.grasp_posture.joint_names = joint_names
    moveit_grasp.grasp_posture.points.append(goal_point)

    # # The position of the end-effector for the grasp.  This is the pose of
    # # the "parent_link" of the end-effector, not actually the pose of any
    # # link *in* the end-effector.  Typically this would be the pose of the
    # # most distal wrist link before the hand (end-effector) links began.
    #
    # geometry_msgs/PoseStamped grasp_pose
    #
    ee_pose = graspit_grasp_pose_to_moveit_grasp_pose(move_group_commander, listener, graspit_grasp_msg, grasp_tran_frame_name)
    moveit_grasp.grasp_pose.pose = ee_pose
    moveit_grasp.grasp_pose.header.frame_id = graspit_grasp_msg.object_name

    # # The estimated probability of success for this grasp, or some other
    # # measure of how "good" it is.
    #
    # float64 grasp_quality
    #
    moveit_grasp.grasp_quality = .8

    # # The approach direction to take before picking an object
    #
    # GripperTranslation pre_grasp_approach
    #
    approach_dir = geometry_msgs.msg.Vector3Stamped()

    x = rospy.get_param('approach_dir_x', 0)
    y = rospy.get_param('approach_dir_y', 0)
    z = rospy.get_param('approach_dir_z', 1)

    approach_dir.vector = geometry_msgs.msg.Vector3(x, y, z)
    approach_dir.header.frame_id = grasp_tran_frame_name
    moveit_grasp.pre_grasp_approach.direction = get_approach_dir_in_ee_coords(move_group_commander, listener, approach_dir)
    #Convert the grasp message to a transform
    grasp_tran = pm.toMatrix(pm.fromMsg(graspit_grasp_msg.final_grasp_pose))


    pregrasp_tran = pm.toMatrix(pm.fromMsg(graspit_grasp_msg.pre_grasp_pose))


    pregrasp_dist = 0.05#np.linalg.norm(pregrasp_tran[0:3, 3] - grasp_tran[0:3, 3])
    moveit_grasp.pre_grasp_approach.desired_distance = 2*pregrasp_dist
    moveit_grasp.pre_grasp_approach.min_distance = pregrasp_dist

    # # The retreat direction to take after a grasp has been completed (object is attached)
    #
    # GripperTranslation post_grasp_retreat
    #
    moveit_grasp.post_grasp_retreat.min_distance = .05
    moveit_grasp.post_grasp_retreat.desired_distance = 2*moveit_grasp.post_grasp_retreat.min_distance
    moveit_grasp.post_grasp_retreat.direction.header.frame_id = '/world'
    moveit_grasp.post_grasp_retreat.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)

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
    moveit_grasp.max_contact_force = -1

    # # an optional list of obstacles that we have semantic information about
    # # and that can be touched/pushed/moved in the course of grasping
    #
    # string[] allowed_touch_objects
    #
    moveit_grasp.allowed_touch_objects = []

    return moveit_grasp
# TODO:
# need pre-grasp-pose
def graspit_interface_grasp_to_moveit_grasp(grasp_id, object_name, graspit_interface_grasp_msg, move_group_commander, listener,  grasp_tran_frame_name='approach_tran'):
    """
    :param graspit_interface_grasp_msg: A graspit grasp message
    :type graspit_interface_grasp_msg: graspit_msgs.msg.Grasp
    :returns a moveit message built from the graspit grasp
    :rtype: moveit_msgs.msg.Grasp
    """
    # 'manipulator' is the name for the root move group in the mico arm
    moveit_positions_from_graspit_positions = {'StaubliArm': barrett_positions_from_graspit_positions,
                                               'manipulator': mico_positions_from_graspit_positions,
                                               'arm': fetch_positions_from_graspit_positions,
                                               'left_arm': pr2_positions_from_graspit_positions}
    move_group_name = rospy.get_param('/move_group_name')
    moveit_positions_from_graspit_positions_fcn = moveit_positions_from_graspit_positions[move_group_name]

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
    moveit_grasp.id = 'graspit_' + str(grasp_id)

    # # The internal posture of the hand for the pre-grasp
    # # only positions are used
    #
    # trajectory_msgs/JointTrajectory pre_grasp_posture
    #
    pre_grasp_goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    spread_pregrasp_dof = (0, 0, 0, 0)
    pre_grasp_joint_names, pre_grasp_goal_point.positions = moveit_positions_from_graspit_positions_fcn(spread_pregrasp_dof)
    moveit_grasp.pre_grasp_posture.points.append(pre_grasp_goal_point)
    moveit_grasp.pre_grasp_posture.joint_names = pre_grasp_joint_names

    # # The internal posture of the hand for the grasp
    # # positions and efforts are used
    #
    # trajectory_msgs/JointTrajectory grasp_posture
    #
    goal_point = trajectory_msgs.msg.JointTrajectoryPoint()
    pre_grasp_dof = (0,0)
    joint_names, goal_point.positions = moveit_positions_from_graspit_positions_fcn(pre_grasp_dof)
    moveit_grasp.grasp_posture.joint_names = joint_names
    moveit_grasp.grasp_posture.points.append(goal_point)

    # # The position of the end-effector for the grasp.  This is the pose of
    # # the "parent_link" of the end-effector, not actually the pose of any
    # # link *in* the end-effector.  Typically this would be the pose of the
    # # most distal wrist link before the hand (end-effector) links began.
    #
    # geometry_msgs/PoseStamped grasp_pose
    #
    ee_pose = graspit_interface_grasp_pose_to_moveit_grasp_pose(move_group_commander, listener, graspit_interface_grasp_msg, grasp_tran_frame_name)
    moveit_grasp.grasp_pose.pose = ee_pose
    moveit_grasp.grasp_pose.header.frame_id = object_name

    # # The estimated probability of success for this grasp, or some other
    # # measure of how "good" it is.
    #
    # float64 grasp_quality
    #
    moveit_grasp.grasp_quality = .8

    # # The approach direction to take before picking an object
    #
    # GripperTranslation pre_grasp_approach
    #
    approach_dir = geometry_msgs.msg.Vector3Stamped()

    x = rospy.get_param('approach_dir_x', 0)
    y = rospy.get_param('approach_dir_y', 0)
    z = rospy.get_param('approach_dir_z', 1)

    approach_dir.vector = geometry_msgs.msg.Vector3(x, y, z)
    approach_dir.header.frame_id = grasp_tran_frame_name

    moveit_grasp.pre_grasp_approach.direction = get_approach_dir_in_ee_coords(move_group_commander, listener, approach_dir)
    moveit_grasp.pre_grasp_approach.desired_distance = 0.05
    moveit_grasp.pre_grasp_approach.min_distance = 0.05

    # # The retreat direction to take after a grasp has been completed (object is attached)
    #
    # GripperTranslation post_grasp_retreat
    #
    moveit_grasp.post_grasp_retreat.min_distance = .05
    moveit_grasp.post_grasp_retreat.desired_distance = 2*moveit_grasp.post_grasp_retreat.min_distance
    moveit_grasp.post_grasp_retreat.direction.header.frame_id = '/world'
    moveit_grasp.post_grasp_retreat.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)

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
    moveit_grasp.max_contact_force = -1

    # # an optional list of obstacles that we have semantic information about
    # # and that can be touched/pushed/moved in the course of grasping
    #
    # string[] allowed_touch_objects
    #
    moveit_grasp.allowed_touch_objects = []

    return moveit_grasp

def build_pickup_goal(moveit_grasp_msg, object_name, planning_group, allowed_planning_time):
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
    pickup_goal.support_surface_name = "table"

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
    # pickup_goal.attached_object_touch_links = ['barrett_mount_link', 'approach_tran', 'staubli_rx60l_link7',
    #                                            'wam/bhand/bhand_palm_link',
    #                                            'wrist_load_cell',
    #                                             'wam/bhand/finger_1/dist_link',
    #                                             'wam/bhand/finger_1/med_link',
    #                                             'wam/bhand/finger_1/prox_link',
    #                                             'wam/bhand/finger_2/dist_link',
    #                                             'wam/bhand/finger_2/med_link',
    #                                             'wam/bhand/finger_2/prox_link',
    #                                             'wam/bhand/finger_3/dist_link',
    #                                             'wam/bhand/finger_3/med_link',
    #                                             'wam/bhand/finger_3/prox_link']

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

    # # an optional list of obstacles that we have semantic information about
    # # and that can be touched/pushed/moved in the course of grasping;
    # # CAREFUL: If the object name 'all' is used, collisions with all objects are disabled during the approach & lift.
    #
    # string[] allowed_touch_objects
    #
    #pickup_goal.allowed_touch_objects = ['all']

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
