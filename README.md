# reachability_analyzer
code for checking grasp reachability

The following parameters are used to convert a graspit_interface grasp message into a moveit pick-place request.  They can be defined in a launch file as follows:
```
<launch>
  <group ns="reachability">

    <param name="analyze_grasp_topic" value="analyze_grasp_action" />
    <param name="move_group_name" value="arm" />
    <param name="end_effector_name" value="gripper" />
 
    <param name="reachability_analyzer/planner_config_name" value="[PRMkConfigDefault]" />
    <param name="reachability_analyzer/allowed_planning_time" value="15" />

    <param name="pre_grasp_goal_point.effort" value="[50,50]" />
    <param name="pre_grasp_goal_point.positions" value="[0.05, 0.05]" />
    <param name="pre_grasp_goal_point.time_from_start.secs" value="0" />

    <param name="pre_grasp_joint_names" value="['l_gripper_finger_joint', 'r_gripper_finger_joint']" />

    <param name="grasp_goal_point.effort" value="[50,50]" />
    <param name="grasp_goal_point.time_from_start.secs" value="0" />

    <param name="moveit_grasp.grasp_posture.joint_names" value="['l_gripper_finger_joint', 'r_gripper_finger_joint']" />

    <param name="moveit_grasp.pre_grasp_approach.min_distance" value="0.05" />
    <param name="moveit_grasp.pre_grasp_approach.desired_distance" value="0.1" />
    <param name="moveit_grasp.pre_grasp_approach.direction.header.frame_id" value="/wrist_roll_link" />

    <param name="moveit_grasp.post_grasp_retreat.min_distance" value="0.05" />
    <param name="moveit_grasp.post_grasp_retreat.desired_distance" value="0.1" />
    <param name="moveit_grasp.post_grasp_retreat.direction.header.frame_id" value="/base_link" />
    <param name="moveit_grasp.post_grasp_retreat.direction.vector.x" value="0" />
    <param name="moveit_grasp.post_grasp_retreat.direction.vector.y" value="0" />
    <param name="moveit_grasp.post_grasp_retreat.direction.vector.z" value="1" />

    <param name="moveit_grasp.max_contact_force" value="-1" />

    <node name="reachability_analyzer" pkg="reachability_analyzer" type="grasp_analyzer_node.py"/>
  </group>
</launch>

```