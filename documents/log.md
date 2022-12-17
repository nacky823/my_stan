# log 置き場

## crane_x7_bringup の demo.launch 立ち上げ時のログ

```
$ roslaunch crane_x7_bringup demo.launch fake_execution:=false 
```

```
... logging to /home/nacky/.ros/log/aa579856-7dd4-11ed-9556-e37e0d570546/roslaunch-nacky-GF65-Thin-10SER-94937.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

xacro: in-order processing became default in ROS Melodic. You can drop the option.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://localhost:41631/

SUMMARY
========

PARAMETERS
 * /crane_x7/arm_controller/constraints/goal_time: 0.0
 * /crane_x7/arm_controller/constraints/stopped_velocity_tolerance: 1.0
 * /crane_x7/arm_controller/joints: ['crane_x7_should...
 * /crane_x7/arm_controller/publish_rate: 250
 * /crane_x7/arm_controller/type: position_controll...
 * /crane_x7/dynamixel_port/baud_rate: 3000000
 * /crane_x7/dynamixel_port/crane_x7_gripper_finger_a_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_gripper_finger_a_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_gripper_finger_a_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_gripper_finger_a_joint/id: 9
 * /crane_x7/dynamixel_port/crane_x7_gripper_finger_a_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_fixed_part_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_fixed_part_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_fixed_part_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_fixed_part_joint/id: 6
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_fixed_part_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_revolute_part_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_revolute_part_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_revolute_part_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_revolute_part_joint/id: 7
 * /crane_x7/dynamixel_port/crane_x7_lower_arm_revolute_part_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_shoulder_fixed_part_pan_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_shoulder_fixed_part_pan_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_shoulder_fixed_part_pan_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_shoulder_fixed_part_pan_joint/id: 2
 * /crane_x7/dynamixel_port/crane_x7_shoulder_fixed_part_pan_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_shoulder_revolute_part_tilt_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_shoulder_revolute_part_tilt_joint/effort_const: 2.79
 * /crane_x7/dynamixel_port/crane_x7_shoulder_revolute_part_tilt_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_shoulder_revolute_part_tilt_joint/id: 3
 * /crane_x7/dynamixel_port/crane_x7_shoulder_revolute_part_tilt_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_rotate_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_rotate_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_rotate_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_rotate_joint/id: 5
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_rotate_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_twist_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_twist_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_twist_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_twist_joint/id: 4
 * /crane_x7/dynamixel_port/crane_x7_upper_arm_revolute_part_twist_joint/mode: 3
 * /crane_x7/dynamixel_port/crane_x7_wrist_joint/center: 2048
 * /crane_x7/dynamixel_port/crane_x7_wrist_joint/effort_const: 1.79
 * /crane_x7/dynamixel_port/crane_x7_wrist_joint/home: 2048
 * /crane_x7/dynamixel_port/crane_x7_wrist_joint/id: 8
 * /crane_x7/dynamixel_port/crane_x7_wrist_joint/mode: 3
 * /crane_x7/dynamixel_port/joints: ['crane_x7_should...
 * /crane_x7/dynamixel_port/port_name: /dev/ttyUSB0
 * /crane_x7/gripper_controller/action_monitor_rate: 10
 * /crane_x7/gripper_controller/goal_tolerance: 0.2
 * /crane_x7/gripper_controller/joint: crane_x7_gripper_...
 * /crane_x7/gripper_controller/publish_rate: 250
 * /crane_x7/gripper_controller/stall_timeout: 0.3
 * /crane_x7/gripper_controller/stall_velocity_threshold: 0.01
 * /crane_x7/gripper_controller/state_publish_rate: 100
 * /crane_x7/gripper_controller/type: position_controll...
 * /crane_x7/joint_limits/arm_joint1/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint1/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint1/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint1/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint1/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint1/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint1/max_position: 1.5707963268
 * /crane_x7/joint_limits/arm_joint1/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint1/min_position: -1.5707963268
 * /crane_x7/joint_limits/arm_joint2/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint2/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint2/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint2/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint2/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint2/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint2/max_position: 0.0
 * /crane_x7/joint_limits/arm_joint2/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint2/min_position: -1.5707963268
 * /crane_x7/joint_limits/arm_joint3/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint3/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint3/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint3/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint3/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint3/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint3/max_position: 1.5707963268
 * /crane_x7/joint_limits/arm_joint3/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint3/min_position: -1.5707963268
 * /crane_x7/joint_limits/arm_joint4/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint4/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint4/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint4/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint4/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint4/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint4/max_position: 2.726204
 * /crane_x7/joint_limits/arm_joint4/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint4/min_position: 0.0
 * /crane_x7/joint_limits/arm_joint5/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint5/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint5/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint5/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint5/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint5/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint5/max_position: 1.5707963268
 * /crane_x7/joint_limits/arm_joint5/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint5/min_position: -1.5707963268
 * /crane_x7/joint_limits/arm_joint6/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint6/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint6/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint6/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint6/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint6/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint6/max_position: 1.047196
 * /crane_x7/joint_limits/arm_joint6/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint6/min_position: -2.094395
 * /crane_x7/joint_limits/arm_joint7/has_acceleration_limits: False
 * /crane_x7/joint_limits/arm_joint7/has_effort_limits: True
 * /crane_x7/joint_limits/arm_joint7/has_jerk_limits: False
 * /crane_x7/joint_limits/arm_joint7/has_position_limits: True
 * /crane_x7/joint_limits/arm_joint7/has_velocity_limits: True
 * /crane_x7/joint_limits/arm_joint7/max_effort: 4.0
 * /crane_x7/joint_limits/arm_joint7/max_position: 2.96706
 * /crane_x7/joint_limits/arm_joint7/max_velocity: 5.969211435
 * /crane_x7/joint_limits/arm_joint7/min_position: -2.96706
 * /crane_x7/joint_limits/hand_joint1/has_acceleration_limits: False
 * /crane_x7/joint_limits/hand_joint1/has_effort_limits: True
 * /crane_x7/joint_limits/hand_joint1/has_jerk_limits: False
 * /crane_x7/joint_limits/hand_joint1/has_position_limits: True
 * /crane_x7/joint_limits/hand_joint1/has_velocity_limits: True
 * /crane_x7/joint_limits/hand_joint1/max_effort: 4.0
 * /crane_x7/joint_limits/hand_joint1/max_position: 0.523598776
 * /crane_x7/joint_limits/hand_joint1/max_velocity: 5.969211435
 * /crane_x7/joint_limits/hand_joint1/min_position: 0.0
 * /crane_x7/joint_state_controller/publish_rate: 250
 * /crane_x7/joint_state_controller/type: joint_state_contr...
 * /move_group/allow_trajectory_execution: True
 * /move_group/arm/planner_configs: ['SBLkConfigDefau...
 * /move_group/controller_list: [{'name': '/crane...
 * /move_group/gripper/longest_valid_segment_fraction: 0.005
 * /move_group/gripper/planner_configs: ['SBLkConfigDefau...
 * /move_group/gripper/projection_evaluator: joints(crane_x7_g...
 * /move_group/jiggle_fraction: 0.05
 * /move_group/max_range: 5.0
 * /move_group/max_safe_path_cost: 1
 * /move_group/moveit_controller_manager: moveit_simple_con...
 * /move_group/moveit_manage_controllers: True
 * /move_group/octomap_resolution: 0.025
 * /move_group/planner_configs/BFMTkConfigDefault/balanced: 0
 * /move_group/planner_configs/BFMTkConfigDefault/cache_cc: 1
 * /move_group/planner_configs/BFMTkConfigDefault/extended_fmt: 1
 * /move_group/planner_configs/BFMTkConfigDefault/heuristics: 1
 * /move_group/planner_configs/BFMTkConfigDefault/nearest_k: 1
 * /move_group/planner_configs/BFMTkConfigDefault/num_samples: 1000
 * /move_group/planner_configs/BFMTkConfigDefault/optimality: 1
 * /move_group/planner_configs/BFMTkConfigDefault/radius_multiplier: 1.0
 * /move_group/planner_configs/BFMTkConfigDefault/type: geometric::BFMT
 * /move_group/planner_configs/BKPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/BKPIECEkConfigDefault/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/BKPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/BKPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/BKPIECEkConfigDefault/type: geometric::BKPIECE
 * /move_group/planner_configs/BiESTkConfigDefault/range: 0.0
 * /move_group/planner_configs/BiESTkConfigDefault/type: geometric::BiEST
 * /move_group/planner_configs/BiTRRTkConfigDefault/cost_threshold: 1e300
 * /move_group/planner_configs/BiTRRTkConfigDefault/frountier_node_ratio: 0.1
 * /move_group/planner_configs/BiTRRTkConfigDefault/frountier_threshold: 0.0
 * /move_group/planner_configs/BiTRRTkConfigDefault/init_temperature: 100
 * /move_group/planner_configs/BiTRRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/BiTRRTkConfigDefault/temp_change_factor: 0.1
 * /move_group/planner_configs/BiTRRTkConfigDefault/type: geometric::BiTRRT
 * /move_group/planner_configs/ESTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/ESTkConfigDefault/range: 0.0
 * /move_group/planner_configs/ESTkConfigDefault/type: geometric::EST
 * /move_group/planner_configs/FMTkConfigDefault/cache_cc: 1
 * /move_group/planner_configs/FMTkConfigDefault/extended_fmt: 1
 * /move_group/planner_configs/FMTkConfigDefault/heuristics: 0
 * /move_group/planner_configs/FMTkConfigDefault/nearest_k: 1
 * /move_group/planner_configs/FMTkConfigDefault/num_samples: 1000
 * /move_group/planner_configs/FMTkConfigDefault/radius_multiplier: 1.1
 * /move_group/planner_configs/FMTkConfigDefault/type: geometric::FMT
 * /move_group/planner_configs/KPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/KPIECEkConfigDefault/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/KPIECEkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/KPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/KPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/KPIECEkConfigDefault/type: geometric::KPIECE
 * /move_group/planner_configs/LBKPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/LBKPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/LBKPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/LBKPIECEkConfigDefault/type: geometric::LBKPIECE
 * /move_group/planner_configs/LBTRRTkConfigDefault/epsilon: 0.4
 * /move_group/planner_configs/LBTRRTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/LBTRRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/LBTRRTkConfigDefault/type: geometric::LBTRRT
 * /move_group/planner_configs/LazyPRMkConfigDefault/range: 0.0
 * /move_group/planner_configs/LazyPRMkConfigDefault/type: geometric::LazyPRM
 * /move_group/planner_configs/LazyPRMstarkConfigDefault/type: geometric::LazyPR...
 * /move_group/planner_configs/PDSTkConfigDefault/type: geometric::PDST
 * /move_group/planner_configs/PRMkConfigDefault/max_nearest_neighbors: 10
 * /move_group/planner_configs/PRMkConfigDefault/type: geometric::PRM
 * /move_group/planner_configs/PRMstarkConfigDefault/type: geometric::PRMstar
 * /move_group/planner_configs/ProjESTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/ProjESTkConfigDefault/range: 0.0
 * /move_group/planner_configs/ProjESTkConfigDefault/type: geometric::ProjEST
 * /move_group/planner_configs/RRTConnectkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTConnectkConfigDefault/type: geometric::RRTCon...
 * /move_group/planner_configs/RRTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/RRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTkConfigDefault/type: geometric::RRT
 * /move_group/planner_configs/RRTstarkConfigDefault/delay_collision_checking: 1
 * /move_group/planner_configs/RRTstarkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/RRTstarkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTstarkConfigDefault/type: geometric::RRTstar
 * /move_group/planner_configs/SBLkConfigDefault/range: 0.0
 * /move_group/planner_configs/SBLkConfigDefault/type: geometric::SBL
 * /move_group/planner_configs/SPARSkConfigDefault/dense_delta_fraction: 0.001
 * /move_group/planner_configs/SPARSkConfigDefault/max_failures: 1000
 * /move_group/planner_configs/SPARSkConfigDefault/sparse_delta_fraction: 0.25
 * /move_group/planner_configs/SPARSkConfigDefault/stretch_factor: 3.0
 * /move_group/planner_configs/SPARSkConfigDefault/type: geometric::SPARS
 * /move_group/planner_configs/SPARStwokConfigDefault/dense_delta_fraction: 0.001
 * /move_group/planner_configs/SPARStwokConfigDefault/max_failures: 5000
 * /move_group/planner_configs/SPARStwokConfigDefault/sparse_delta_fraction: 0.25
 * /move_group/planner_configs/SPARStwokConfigDefault/stretch_factor: 3.0
 * /move_group/planner_configs/SPARStwokConfigDefault/type: geometric::SPARStwo
 * /move_group/planner_configs/STRIDEkConfigDefault/degree: 16
 * /move_group/planner_configs/STRIDEkConfigDefault/estimated_dimension: 0.0
 * /move_group/planner_configs/STRIDEkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/STRIDEkConfigDefault/max_degree: 18
 * /move_group/planner_configs/STRIDEkConfigDefault/max_pts_per_leaf: 6
 * /move_group/planner_configs/STRIDEkConfigDefault/min_degree: 12
 * /move_group/planner_configs/STRIDEkConfigDefault/min_valid_path_fraction: 0.2
 * /move_group/planner_configs/STRIDEkConfigDefault/range: 0.0
 * /move_group/planner_configs/STRIDEkConfigDefault/type: geometric::STRIDE
 * /move_group/planner_configs/STRIDEkConfigDefault/use_projected_distance: 0
 * /move_group/planner_configs/TRRTkConfigDefault/frountierNodeRatio: 0.1
 * /move_group/planner_configs/TRRTkConfigDefault/frountier_threshold: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/TRRTkConfigDefault/init_temperature: 10e-6
 * /move_group/planner_configs/TRRTkConfigDefault/k_constant: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/max_states_failed: 10
 * /move_group/planner_configs/TRRTkConfigDefault/min_temperature: 10e-10
 * /move_group/planner_configs/TRRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/temp_change_factor: 2.0
 * /move_group/planner_configs/TRRTkConfigDefault/type: geometric::TRRT
 * /move_group/planning_plugin: ompl_interface/OM...
 * /move_group/planning_scene_monitor/publish_geometry_updates: True
 * /move_group/planning_scene_monitor/publish_planning_scene: True
 * /move_group/planning_scene_monitor/publish_state_updates: True
 * /move_group/planning_scene_monitor/publish_transforms_updates: True
 * /move_group/request_adapters: default_planner_r...
 * /move_group/start_state_max_bounds_error: 0.1
 * /move_group/trajectory_execution/allowed_execution_duration_scaling: 0.6
 * /move_group/trajectory_execution/allowed_goal_duration_margin: 0.5
 * /move_group/trajectory_execution/allowed_start_tolerance: 0.1
 * /robot_description: <?xml version="1....
 * /robot_description_kinematics/arm/kinematics_solver: kdl_kinematics_pl...
 * /robot_description_kinematics/arm/kinematics_solver_search_resolution: 0.005
 * /robot_description_kinematics/arm/kinematics_solver_timeout: 0.005
 * /robot_description_planning/default_acceleration_scaling_factor: 1.0
 * /robot_description_planning/default_velocity_scaling_factor: 1.0
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_a_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_a_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_a_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_a_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_b_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_b_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_b_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_gripper_finger_b_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_fixed_part_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_fixed_part_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_fixed_part_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_fixed_part_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_revolute_part_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_revolute_part_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_revolute_part_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_lower_arm_revolute_part_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_shoulder_fixed_part_pan_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_shoulder_fixed_part_pan_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_shoulder_fixed_part_pan_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_shoulder_fixed_part_pan_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_shoulder_revolute_part_tilt_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_shoulder_revolute_part_tilt_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_shoulder_revolute_part_tilt_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_shoulder_revolute_part_tilt_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_rotate_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_rotate_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_rotate_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_rotate_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_twist_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_twist_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_twist_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_upper_arm_revolute_part_twist_joint/max_velocity: 4.81710873
 * /robot_description_planning/joint_limits/crane_x7_wrist_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/crane_x7_wrist_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/crane_x7_wrist_joint/max_acceleration: 4.4
 * /robot_description_planning/joint_limits/crane_x7_wrist_joint/max_velocity: 4.81710873
 * /robot_description_semantic: <?xml version="1....
 * /rosdistro: noetic
 * /rosversion: 1.15.14
 * /rviz_nacky_GF65_Thin_10SER_94937_8977060866828422126/arm/kinematics_solver: kdl_kinematics_pl...
 * /rviz_nacky_GF65_Thin_10SER_94937_8977060866828422126/arm/kinematics_solver_search_resolution: 0.005
 * /rviz_nacky_GF65_Thin_10SER_94937_8977060866828422126/arm/kinematics_solver_timeout: 0.005
 * /source_list: ['/crane_x7/joint...

NODES
  /
    joint_state_publisher (joint_state_publisher/joint_state_publisher)
    move_group (moveit_ros_move_group/move_group)
    rviz_nacky_GF65_Thin_10SER_94937_8977060866828422126 (rviz/rviz)
  /crane_x7/
    controller_manager (controller_manager/spawner)
    crane_x7_control (crane_x7_control/crane_x7_control)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)

auto-starting new master
process[master]: started with pid [94982]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to aa579856-7dd4-11ed-9556-e37e0d570546
process[rosout-1]: started with pid [95003]
started core service [/rosout]
process[joint_state_publisher-2]: started with pid [95010]
process[crane_x7/controller_manager-3]: started with pid [95011]
process[crane_x7/crane_x7_control-4]: started with pid [95012]
process[crane_x7/robot_state_publisher-5]: started with pid [95013]
process[move_group-6]: started with pid [95014]
process[rviz_nacky_GF65_Thin_10SER_94937_8977060866828422126-7]: started with pid [95016]
[PortHandlerLinux::SetupPort] Error opening serial port!
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/rospack_nosubdirs: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/rospack_nosubdirs"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/manifest.xml: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/manifest.xml"
[ WARN] [1671258787.556278791]: Falling back to using the move_group node's namespace (deprecated Melodic behavior).
[ INFO] [1671258787.566295479]: Loading robot model 'crane_x7'...
[ INFO] [1671258787.566352197]: No root/virtual joint specified in SRDF. Assuming fixed joint
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/rospack_nosubdirs: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/rospack_nosubdirs"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/manifest.xml: boost::filesystem::status: Permission denied: "/opt/ros/noetic/share/nacky/snap/manifest.xml"
[ INFO] [1671258787.588652971]: SKIP SELF CHECK...
[ INFO] [1671258787.685208162]: rviz version 1.14.19
[ INFO] [1671258787.685255565]: compiled against Qt version 5.12.8
[ INFO] [1671258787.685268118]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1671258787.694910486]: Forcing OpenGl version 0.
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE: boost::filesystem::status: 許可がありません: "/opt/ros/noetic/share/nacky/snap/CATKIN_IGNORE"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/rospack_nosubdirs: boost::filesystem::status: 許可がありません: "/opt/ros/noetic/share/nacky/snap/rospack_nosubdirs"
[rospack] Warning: error while looking for /opt/ros/noetic/share/nacky/snap/manifest.xml: boost::filesystem::status: 許可がありません: "/opt/ros/noetic/share/nacky/snap/manifest.xml"
[ INFO] [1671258787.719713297]: Publishing maintained planning scene on 'monitored_planning_scene'
[ INFO] [1671258787.722178809]: Listening to 'joint_states' for joint states
[ INFO] [1671258787.726344398]: Listening to '/attached_collision_object' for attached collision objects
[ INFO] [1671258787.726366342]: Starting planning scene monitor
[ INFO] [1671258787.728194360]: Listening to '/planning_scene'
[ INFO] [1671258787.728212710]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[ INFO] [1671258787.729958555]: Listening to '/collision_object'
[ INFO] [1671258787.732211368]: Listening to '/planning_scene_world' for planning scene world geometry
[ INFO] [1671258787.732870270]: No 3D sensor plugin(s) defined for octomap updates
[ INFO] [1671258787.738026671]: Loading planning pipeline ''
[ INFO] [1671258787.782080601]: Using planning interface 'OMPL'
[ INFO] [1671258787.784023491]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1671258787.784314214]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1671258787.784567867]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1671258787.784828136]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1671258787.785076701]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1671258787.785338195]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1671258787.785367776]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1671258787.785379936]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1671258787.785388648]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1671258787.785396299]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1671258787.785404273]: Using planning request adapter 'Fix Start State Path Constraints'
[ INFO] [1671258787.874898395]: Stereo is NOT SUPPORTED
[ INFO] [1671258787.874945190]: OpenGL device: Mesa Intel(R) UHD Graphics (CML GT2)
[ INFO] [1671258787.874964099]: OpenGl version: 4.6 (GLSL 4.6) limited to GLSL 1.4 on Mesa system.
[INFO] [1671258787.904809]: Controller Spawner: Waiting for service controller_manager/load_controller
[INFO] [1671258787.908361]: Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [1671258787.909756]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1671258787.911343]: Loading controller: joint_state_controller
[INFO] [1671258787.920628]: Loading controller: arm_controller
[INFO] [1671258787.945452]: Loading controller: gripper_controller
[INFO] [1671258787.960454]: Controller Spawner: Loaded controllers: joint_state_controller, arm_controller, gripper_controller
[INFO] [1671258787.965351]: Started controllers: joint_state_controller, arm_controller, gripper_controller
[ INFO] [1671258788.154110767]: Added FollowJointTrajectory controller for /crane_x7/arm_controller
[ INFO] [1671258788.458058164]: Added GripperCommand controller for /crane_x7/gripper_controller
[ INFO] [1671258788.458176458]: Returned 2 controllers in list
[ INFO] [1671258788.464919169]: Trajectory execution is managing controllers
[ INFO] [1671258788.464970633]: MoveGroup debug mode is ON
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
[ INFO] [1671258788.498439177]:

********************************************************
* MoveGroup using:
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
********************************************************

[ INFO] [1671258788.498683290]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1671258788.498698672]: MoveGroup context initialization complete

You can start planning now!

[ INFO] [1671258791.117109988]: Loading robot model 'crane_x7'...
[ INFO] [1671258791.117166107]: No root/virtual joint specified in SRDF. Assuming fixed joint
[ INFO] [1671258791.207355447]: Starting planning scene monitor
[ INFO] [1671258791.209014519]: Listening to '/move_group/monitored_planning_scene'
[ INFO] [1671258791.404706694]: Constructing new MoveGroup connection for group 'arm' in namespace ''
[ INFO] [1671258792.566675152]: Ready to take commands for planning group arm.
```

