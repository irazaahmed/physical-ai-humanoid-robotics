---
doc_id: module3_m3_ch06
title: "Navigation and Path Planning with Nav2"
module: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
estimated_tokens: 1250
embedding_required: true
chunk_hint_tokens: 500
---

# Navigation and Path Planning with Nav2

## Objective

This chapter introduces students to navigation and path planning concepts using the Navigation2 (Nav2) stack integrated with NVIDIA Isaac. Students will learn how to configure, optimize, and deploy navigation systems for autonomous mobile robots in the Isaac ecosystem.

## Learning Outcomes

After completing this chapter, students will be able to:
1. Configure Nav2 navigation stack for mobile robots in Isaac simulation
2. Implement global and local path planning with GPU-accelerated components
3. Integrate Nav2 with Isaac Sim for comprehensive navigation testing
4. Evaluate navigation performance and troubleshoot common issues

## Theory

Navigation is a fundamental capability for autonomous mobile robots, enabling them to move safely and efficiently from one location to another in known or unknown environments. The Navigation2 (Nav2) stack is the current ROS 2-based navigation framework that provides a complete solution for path planning, obstacle avoidance, and trajectory execution.

### Navigation Architecture

The Nav2 stack consists of several key components that work together to provide complete navigation functionality:

1. **Global Planner**: Computes the optimal path from the robot's current location to the goal location using the global map
2. **Local Planner**: Tracks the global path while avoiding dynamic obstacles and adjusting for robot kinematics
3. **Controller**: Converts path-following commands into velocity commands for the robot's actuators
4. **Costmaps**: Maintains spatial representations of the environment for path planning and obstacle avoidance
5. **Recovery Behaviors**: Handles situations where the robot becomes stuck or unable to progress

### Nav2 Components

Nav2 uses a behavior tree architecture that allows for flexible and configurable navigation behaviors. The core components include:

1. **Navigator**: Manages the navigation lifecycle and coordinates between components
2. **Planners**: Implement various path planning algorithms (A*, Dijkstra, etc.)
3. **Controllers**: Execute local path following (Trajectory Control, DWA, etc.)
4. **Behavior Trees**: Orchestrate navigation behaviors and handle transitions between states

### Isaac Integration Benefits

Integrating Nav2 with Isaac provides several advantages:

1. **High-Fidelity Simulation**: Isaac Sim provides realistic physics and sensor simulation for navigation testing
2. **GPU Acceleration**: Isaac's GPU capabilities can accelerate perception and planning components
3. **Photorealistic Environments**: Enables testing with synthetic data that closely matches real-world conditions
4. **Domain Randomization**: Helps improve sim-to-real transfer by varying environmental conditions

### Global vs Local Planning

Navigation systems typically employ a hierarchical approach with:
- **Global Planning**: Finds the optimal path from start to goal in a static environment using a known map. Algorithms like A* and Dijkstra's provide efficient pathfinding capabilities.
- **Local Planning**: Tracks the global path while avoiding dynamic obstacles and adapting to the immediate environment. Local planners must react quickly to sensor inputs and account for robot dynamics and kinematic constraints.

### Costmap Representation

Costmaps represent the robot's perception of the environment as a grid of cells with associated costs. Each cell represents occupancy probability and influences the path planner's decisions. The Nav2 stack maintains separate global and local costmaps:
- **Global Costmap**: Covers a larger area, updated less frequently, used by global planner
- **Local Costmap**: Covers robot's immediate vicinity, updated frequently, used by local planner

## Practical Examples

### Example 1: Basic Nav2 Configuration

A basic Nav2 configuration with Isaac integration:

```yaml
# ~/isaac_nav2_ws/config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: False
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    lidar_topic: "scan"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
   odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::LikelihoodFieldModel"
    save_pose_delay: 0.2
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.5
    update_min_d: 0.2

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_feedback_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      # Inner controller to use
      controller_plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # Rotation shim controller parameters
      simulate_to_lookahead_time: 1.0
      rotational_scaler: 1.0
      max_angular_accel: 3.2
      lookahead_time: 1.5
      min_lookahead_distance: 0.3
      max_lookahead_distance: 0.9

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0