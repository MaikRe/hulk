#![recursion_limit = "256"]
pub mod action;
pub mod audio;
pub mod ball_detection;
pub mod ball_position;
pub mod bounding_box;
pub mod buttons;
pub mod calibration;
pub mod camera_position;
pub mod color;
pub mod condition_input;
pub mod cycle_time;
pub mod detected_feet;
pub mod fall_state;
pub mod field_border;
pub mod field_color;
pub mod field_dimensions;
pub mod field_lines;
pub mod field_marks;
pub mod filtered_game_controller_state;
pub mod filtered_game_state;
pub mod filtered_segments;
pub mod filtered_whistle;
pub mod foot_bumper_obstacle;
pub mod foot_bumper_values;
pub mod game_controller_state;
pub mod grayscale_image;
pub mod hardware;
pub mod image_segments;
pub mod initial_look_around;
pub mod initial_pose;
pub mod joints;
pub mod joints_velocity;
pub mod jpeg;
pub mod kick_decision;
pub mod last_filtered_game_controller_state_change;
pub mod led;
pub mod limb;
pub mod line_data;
pub mod localization;
pub mod message_event;
pub mod messages;
pub mod motion_command;
pub mod motion_selection;
pub mod motor_commands;
pub mod multivariate_normal_distribution;
pub mod obstacle_avoiding_arms;
pub mod obstacle_filter;
pub mod obstacles;
pub mod parameters;
pub mod path_obstacles;
pub mod penalty_shot_direction;
pub mod perspective_grid_candidates;
pub mod planned_path;
pub mod players;
pub mod point_of_interest;
pub mod pose_detection;
pub mod pose_kinds;
pub mod primary_state;
pub mod robot_dimensions;
pub mod robot_kinematics;
pub mod robot_masses;
pub mod roles;
pub mod rule_obstacles;
pub mod samples;
pub mod sensor_data;
pub mod sole_pressure;
pub mod sonar_obstacle;
pub mod sonar_values;
pub mod step_plan;
pub mod support_foot;
pub mod walk_command;
pub mod whistle;
pub mod world_state;
pub mod ycbcr422_image;
