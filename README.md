# reachability_analyzer
code for checking grasp reachability

The following args are used to convert a graspit_interface grasp message into a moveit pick-place request.  They can be defined in a launch file as follows:
```
  <include file="$(find reachability_analyzer)/launch/reachability_analyzer.launch">
    <arg name="analyze_grasp_topic" value="analyze_grasp_action" />
    <arg name="arm_move_group_name" value="StaubliArm" />
    <arg name="hand_move_group_name" value="BarrettHand" />
    <arg name="end_effector_name" value="BarrettHand" />

    <arg name="planner_config_name" value="[PRMkConfigDefault]" />
    <arg name="allowed_planning_time" value="15" />

    <arg name="pre_grasp_goal_point.time_from_start.secs" value="0" />

    <arg name="grasp_goal_point.effort" value="50" />
    <arg name="grasp_goal_point.time_from_start.secs" value="0" />

    <arg name="moveit_grasp.pre_grasp_approach.min_distance" value="0.05" />
    <arg name="moveit_grasp.pre_grasp_approach.desired_distance" value="0.1" />
    <arg name="moveit_grasp.pre_grasp_approach.direction.header.frame_id" value="approach" />
    <arg name="moveit_grasp.pre_grasp_approach.direction.vector.x" value="0" />
    <arg name="moveit_grasp.pre_grasp_approach.direction.vector.y" value="0" />
    <arg name="moveit_grasp.pre_grasp_approach.direction.vector.z" value="1" />

    <arg name="moveit_grasp.post_grasp_retreat.min_distance" value="0.05" />
    <arg name="moveit_grasp.post_grasp_retreat.desired_distance" value="0.1" />
    <arg name="moveit_grasp.post_grasp_retreat.direction.header.frame_id" value="/robot_base" />
    <arg name="moveit_grasp.post_grasp_retreat.direction.vector.x" value="0" />
    <arg name="moveit_grasp.post_grasp_retreat.direction.vector.y" value="0" />
    <arg name="moveit_grasp.post_grasp_retreat.direction.vector.z" value="1" />

    <arg name="moveit_grasp.max_contact_force" value="-1" />
  </include>
```