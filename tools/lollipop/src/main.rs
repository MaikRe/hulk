use clap::Parser;
use color_eyre::eyre::{Result, WrapErr};
use csv::Writer;
use flatten_json_object::Flattener;
use hula_types::RobotState;
use kinematics::{
    head_to_neck, left_ankle_to_left_tibia, left_elbow_to_left_upper_arm, left_foot_to_left_ankle,
    left_forearm_to_left_elbow, left_hip_to_left_pelvis, left_pelvis_to_robot,
    left_shoulder_to_robot, left_thigh_to_left_hip, left_tibia_to_left_thigh,
    left_upper_arm_to_left_shoulder, left_wrist_to_left_forearm, neck_to_robot,
    right_ankle_to_right_tibia, right_elbow_to_right_upper_arm, right_foot_to_right_ankle,
    right_forearm_to_right_elbow, right_hip_to_right_pelvis, right_pelvis_to_robot,
    right_shoulder_to_robot, right_thigh_to_right_hip, right_tibia_to_right_thigh,
    right_upper_arm_to_right_shoulder, right_wrist_to_right_forearm,
};
use log::{debug, LevelFilter};
use nalgebra::{point, vector, Const, Isometry3, OPoint, Point, Point3, Translation, Vector3};

use rmp_serde::from_slice;
use std::{fs::File, io::Read, path::PathBuf};
use types::{
    joints::{arm::ArmJoints, head::HeadJoints, leg::LegJoints, Joints},
    robot_dimensions::RobotDimensions,
    robot_masses::RobotMass,
};
const LOLA_BUFF_SIZE: usize = 896;
#[derive(Parser, Debug)]
#[clap(
    name = "lollipop",
    about = "Open HULA/LoLA passthrough files and convert to csv"
)]
struct Arguments {
    /// Path to file to convert
    file: PathBuf,
    /// Log with Debug log level
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<()> {
    let matches = Arguments::parse();
    env_logger::builder()
        .filter(
            None,
            if matches.verbose {
                LevelFilter::Debug
            } else {
                LevelFilter::Info
            },
        )
        .init();
    let input_file = matches.file.as_path();
    let mut filewriter = Writer::from_path(format!("{}.csv", input_file.display()))?;
    let mut file = File::open(input_file.as_os_str()).wrap_err("Failed to open file")?;
    let mut timestamp_buf = [0; 16];

    let flattener = Flattener::new().set_key_separator(".");
    let mut left_has_more_pressure = false;
    let mut header = true;
    let mut last_timestamp = 0;
    while let Ok(()) = file.read_exact(&mut timestamp_buf) {
        let timestamp = u128::from_be_bytes(timestamp_buf);
        let mut robot_state =
            read_lola_message(&mut file).wrap_err("failed to read lola message")?;
        let left_sum = left_fsr_sum(&robot_state);
        let right_sum = right_fsr_sum(&robot_state);
        left_has_more_pressure = greater_than_with_hysteresis(
            left_has_more_pressure,
            left_fsr_sum(&robot_state),
            right_fsr_sum(&robot_state),
            0.2,
        );
        let AllTheCalculationsFunctionResult {
            center_of_mass,
            center_of_mass_in_parallel,
            accelerometer_in_parallel,
            left_is_support_imu,
            left_sole,
            right_sole,
            center_of_mass_in_ground,
            left_convex_hull,
            right_convex_hull,
            left_hull_index,
            right_hull_index,
            x_zero_moment_point_in_parallel,
            y_zero_moment_point_in_parallel,
        } = all_the_calculations_function(&robot_state, left_has_more_pressure);
        robot_state.received_at = timestamp as f32;
        // debug!("{}", temp_front_convex_left_sole);

        let jsonobject = serde_json::to_value(robot_state).wrap_err("Could not convert to json")?;
        let output = flattener
            .flatten(&jsonobject)
            .wrap_err("Failed to flatten")?;
        if header {
            let mut keys = output
                .as_object()
                .into_iter()
                .flatten()
                .map(|(key, _value)| (key.to_string()))
                .collect::<Vec<_>>();
            keys.push("center_of_mass.x".to_string());
            keys.push("center_of_mass.y".to_string());
            keys.push("center_of_mass.z".to_string());
            keys.push("center_of_mass_in_parallel.x".to_string());
            keys.push("center_of_mass_in_parallel.y".to_string());
            keys.push("center_of_mass_in_parallel.z".to_string());
            keys.push("center_in_ground.x".to_string());
            keys.push("center_in_ground.y".to_string());
            keys.push("center_in_ground.z".to_string());
            keys.push("left_is_support_fsr".to_string());
            keys.push("left_fsr_sum".to_string());
            keys.push("right_fsr_sum".to_string());
            keys.push("accelerometer_in_parallel.x".to_string());
            keys.push("accelerometer_in_parallel.y".to_string());
            keys.push("accelerometer_in_parallel.z".to_string());
            keys.push("left_sole.x".to_string());
            keys.push("left_sole.y".to_string());
            keys.push("left_sole.z".to_string());
            keys.push("right_sole.x".to_string());
            keys.push("right_sole.y".to_string());
            keys.push("right_sole.z".to_string());
            keys.push("left_is_support_imu".to_string());
            keys.push("frame_length".to_string());
            for i in 0..33 {
                keys.push(format!("left_convex.{}.x", i));
                keys.push(format!("left_convex.{}.y", i));
            }
            for i in 0..33 {
                keys.push(format!("right_convex.{}.x", i));
                keys.push(format!("right_convex.{}.y", i));
            }
            for i in 0..33 {
                keys.push(format!("left_hull_index.{}", i));
            }
            for i in 0..33 {
                keys.push(format!("right_hull_index.{}", i));
            }
            keys.push("x_zero_moment_point_in_parallel".to_string());
            keys.push("y_zero_moment_point_in_parallel".to_string());
            debug!("{:?}", keys);
            filewriter.write_record(keys)?;
            header = false;
        }

        //convert robot state to values
        let mut values = output
            .as_object()
            .into_iter()
            .flatten()
            .map(|(_key, value)| (format!("{}", value.as_f64().unwrap())))
            .collect::<Vec<_>>();

        //add all of the other values
        values.push(center_of_mass.x.to_string());
        values.push(center_of_mass.y.to_string());
        values.push(center_of_mass.z.to_string());
        values.push(center_of_mass_in_parallel.x.to_string());
        values.push(center_of_mass_in_parallel.y.to_string());
        values.push(center_of_mass_in_parallel.z.to_string());
        values.push(center_of_mass_in_ground.x.to_string());
        values.push(center_of_mass_in_ground.y.to_string());
        values.push(center_of_mass_in_ground.z.to_string());
        values.push((left_has_more_pressure as i8).to_string());
        values.push(left_sum.to_string());
        values.push(right_sum.to_string());
        values.push(accelerometer_in_parallel.x.to_string());
        values.push(accelerometer_in_parallel.y.to_string());
        values.push(accelerometer_in_parallel.z.to_string());
        values.push(left_sole.x.to_string());
        values.push(left_sole.y.to_string());
        values.push(left_sole.z.to_string());
        values.push(right_sole.x.to_string());
        values.push(right_sole.y.to_string());
        values.push(right_sole.z.to_string());
        values.push((left_is_support_imu as i8).to_string());
        values.push((timestamp - last_timestamp).to_string());
        for i in left_convex_hull {
            values.push(i.x.to_string());
            values.push(i.y.to_string());
        }
        for i in right_convex_hull {
            values.push(i.x.to_string());
            values.push(i.y.to_string());
        }
        for i in left_hull_index {
            values.push((i as i8).to_string());
        }
        for i in right_hull_index {
            values.push((i as i8).to_string());
        }
        values.push(x_zero_moment_point_in_parallel.to_string());
        values.push(y_zero_moment_point_in_parallel.to_string());

        //TODO add check for falling of robot which will potentially remove this frame plus before and after from data

        //write values
        filewriter.write_record(values)?;
        debug!("{}", timestamp);
        last_timestamp = timestamp;
    }

    Ok(())
}
fn read_lola_message(lola: &mut File) -> Result<RobotState> {
    let mut lola_data = [0; LOLA_BUFF_SIZE];
    lola.read_exact(&mut lola_data)
        .wrap_err("failed to read next message pack")?;
    from_slice(&lola_data).wrap_err("failed to parse MessagePack from LoLA StateMessage")
}

pub fn greater_than_with_hysteresis(
    last_evaluation: bool,
    value: f32,
    threshold: f32,
    hysteresis: f32,
) -> bool {
    value
        > threshold
            + if last_evaluation {
                -hysteresis
            } else {
                hysteresis
            }
}

fn left_fsr_sum(robot_state: &RobotState) -> f32 {
    robot_state.force_sensitive_resistors.left_foot_front_left
        + robot_state.force_sensitive_resistors.left_foot_front_right
        + robot_state.force_sensitive_resistors.left_foot_rear_left
        + robot_state.force_sensitive_resistors.left_foot_rear_right
}
fn right_fsr_sum(robot_state: &RobotState) -> f32 {
    robot_state.force_sensitive_resistors.right_foot_front_left
        + robot_state.force_sensitive_resistors.right_foot_front_right
        + robot_state.force_sensitive_resistors.right_foot_rear_left
        + robot_state.force_sensitive_resistors.right_foot_rear_right
}

struct AllTheCalculationsFunctionResult {
    center_of_mass: Point3<f32>,
    center_of_mass_in_parallel: Point3<f32>,
    accelerometer_in_parallel: Point3<f32>,
    left_is_support_imu: bool,
    left_sole: Point3<f32>,
    right_sole: Point3<f32>,
    center_of_mass_in_ground: Point3<f32>,
    left_convex_hull: Vec<OPoint<f32, Const<2>>>,
    right_convex_hull: Vec<OPoint<f32, Const<2>>>,
    left_hull_index: Vec<bool>,
    right_hull_index: Vec<bool>,
    x_zero_moment_point_in_parallel: f32,
    y_zero_moment_point_in_parallel: f32,
}

fn all_the_calculations_function(
    robot_state: &RobotState,
    left_has_more_pressure: bool,
) -> AllTheCalculationsFunctionResult {
    let positions = robot_state.position;
    let joints = Joints {
        head: HeadJoints {
            yaw: positions.head_yaw,
            pitch: positions.head_pitch,
        },
        left_arm: ArmJoints {
            shoulder_pitch: positions.left_shoulder_pitch,
            shoulder_roll: positions.left_shoulder_roll,
            elbow_yaw: positions.left_elbow_yaw,
            elbow_roll: positions.left_elbow_roll,
            wrist_yaw: positions.left_wrist_yaw,
            hand: positions.left_hand,
        },
        right_arm: ArmJoints {
            shoulder_pitch: positions.right_shoulder_pitch,
            shoulder_roll: positions.right_shoulder_roll,
            elbow_yaw: positions.right_elbow_yaw,
            elbow_roll: positions.right_elbow_roll,
            wrist_yaw: positions.right_wrist_yaw,
            hand: positions.right_hand,
        },
        left_leg: LegJoints {
            ankle_pitch: positions.left_ankle_pitch,
            ankle_roll: positions.left_ankle_roll,
            hip_pitch: positions.left_hip_pitch,
            hip_roll: positions.left_hip_roll,
            hip_yaw_pitch: positions.left_hip_yaw_pitch,
            knee_pitch: positions.left_knee_pitch,
        },
        right_leg: LegJoints {
            ankle_pitch: positions.right_ankle_pitch,
            ankle_roll: positions.right_ankle_roll,
            hip_pitch: positions.right_hip_pitch,
            hip_roll: positions.right_hip_roll,
            hip_yaw_pitch: positions.left_hip_yaw_pitch,
            knee_pitch: positions.right_knee_pitch,
        },
    };
    let neck_to_robot = neck_to_robot(&joints.head);
    let head_to_robot = neck_to_robot * head_to_neck(&joints.head);
    // torso
    let torso_to_robot = Isometry3::from(RobotDimensions::ROBOT_TO_TORSO);
    // left arm
    let left_shoulder_to_robot = left_shoulder_to_robot(&joints.left_arm);
    let left_upper_arm_to_robot =
        left_shoulder_to_robot * left_upper_arm_to_left_shoulder(&joints.left_arm);
    let left_elbow_to_robot =
        left_upper_arm_to_robot * left_elbow_to_left_upper_arm(&joints.left_arm);
    let left_forearm_to_robot = left_elbow_to_robot * left_forearm_to_left_elbow(&joints.left_arm);
    let left_wrist_to_robot = left_forearm_to_robot * left_wrist_to_left_forearm(&joints.left_arm);
    // right arm
    let right_shoulder_to_robot = right_shoulder_to_robot(&joints.right_arm);
    let right_upper_arm_to_robot =
        right_shoulder_to_robot * right_upper_arm_to_right_shoulder(&joints.right_arm);
    let right_elbow_to_robot =
        right_upper_arm_to_robot * right_elbow_to_right_upper_arm(&joints.right_arm);
    let right_forearm_to_robot =
        right_elbow_to_robot * right_forearm_to_right_elbow(&joints.right_arm);
    let right_wrist_to_robot =
        right_forearm_to_robot * right_wrist_to_right_forearm(&joints.right_arm);
    // left leg
    let left_pelvis_to_robot = left_pelvis_to_robot(&joints.left_leg);
    let left_hip_to_robot = left_pelvis_to_robot * left_hip_to_left_pelvis(&joints.left_leg);
    let left_thigh_to_robot = left_hip_to_robot * left_thigh_to_left_hip(&joints.left_leg);
    let left_tibia_to_robot = left_thigh_to_robot * left_tibia_to_left_thigh(&joints.left_leg);
    let left_ankle_to_robot = left_tibia_to_robot * left_ankle_to_left_tibia(&joints.left_leg);
    let left_foot_to_robot = left_ankle_to_robot * left_foot_to_left_ankle(&joints.left_leg);
    let left_sole_to_robot = left_foot_to_robot * Translation::from(RobotDimensions::ANKLE_TO_SOLE);
    // right leg
    let right_pelvis_to_robot = right_pelvis_to_robot(&joints.right_leg);
    let right_hip_to_robot = right_pelvis_to_robot * right_hip_to_right_pelvis(&joints.right_leg);
    let right_thigh_to_robot = right_hip_to_robot * right_thigh_to_right_hip(&joints.right_leg);
    let right_tibia_to_robot = right_thigh_to_robot * right_tibia_to_right_thigh(&joints.right_leg);
    let right_ankle_to_robot = right_tibia_to_robot * right_ankle_to_right_tibia(&joints.right_leg);
    let right_foot_to_robot = right_ankle_to_robot * right_foot_to_right_ankle(&joints.right_leg);
    let right_sole_to_robot =
        right_foot_to_robot * Translation::from(RobotDimensions::ANKLE_TO_SOLE);

    //center of mass
    let center_of_mass = (RobotMass::TORSO.mass
        * (torso_to_robot * RobotMass::TORSO.center).coords
        + RobotMass::NECK.mass * (neck_to_robot * RobotMass::NECK.center).coords
        + RobotMass::HEAD.mass * (head_to_robot * RobotMass::HEAD.center).coords
        + RobotMass::LEFT_SHOULDER.mass
            * (left_shoulder_to_robot * RobotMass::LEFT_SHOULDER.center).coords
        + RobotMass::LEFT_UPPER_ARM.mass
            * (left_upper_arm_to_robot * RobotMass::LEFT_UPPER_ARM.center).coords
        + RobotMass::LEFT_ELBOW.mass * (left_elbow_to_robot * RobotMass::LEFT_ELBOW.center).coords
        + RobotMass::LEFT_FOREARM.mass
            * (left_forearm_to_robot * RobotMass::LEFT_FOREARM.center).coords
        + RobotMass::LEFT_WRIST.mass * (left_wrist_to_robot * RobotMass::LEFT_WRIST.center).coords
        + RobotMass::RIGHT_SHOULDER.mass
            * (right_shoulder_to_robot * RobotMass::RIGHT_SHOULDER.center).coords
        + RobotMass::RIGHT_UPPER_ARM.mass
            * (right_upper_arm_to_robot * RobotMass::RIGHT_UPPER_ARM.center).coords
        + RobotMass::RIGHT_ELBOW.mass
            * (right_elbow_to_robot * RobotMass::RIGHT_ELBOW.center).coords
        + RobotMass::RIGHT_FOREARM.mass
            * (right_forearm_to_robot * RobotMass::RIGHT_FOREARM.center).coords
        + RobotMass::RIGHT_WRIST.mass
            * (right_wrist_to_robot * RobotMass::RIGHT_WRIST.center).coords
        + RobotMass::LEFT_PELVIS.mass
            * (left_pelvis_to_robot * RobotMass::LEFT_PELVIS.center).coords
        + RobotMass::LEFT_HIP.mass * (left_hip_to_robot * RobotMass::LEFT_HIP.center).coords
        + RobotMass::LEFT_THIGH.mass * (left_thigh_to_robot * RobotMass::LEFT_THIGH.center).coords
        + RobotMass::LEFT_TIBIA.mass * (left_tibia_to_robot * RobotMass::LEFT_TIBIA.center).coords
        + RobotMass::LEFT_ANKLE.mass * (left_ankle_to_robot * RobotMass::LEFT_ANKLE.center).coords
        + RobotMass::LEFT_FOOT.mass * (left_foot_to_robot * RobotMass::LEFT_FOOT.center).coords
        + RobotMass::RIGHT_PELVIS.mass
            * (right_pelvis_to_robot * RobotMass::RIGHT_PELVIS.center).coords
        + RobotMass::RIGHT_HIP.mass * (right_hip_to_robot * RobotMass::RIGHT_HIP.center).coords
        + RobotMass::RIGHT_THIGH.mass
            * (right_thigh_to_robot * RobotMass::RIGHT_THIGH.center).coords
        + RobotMass::RIGHT_TIBIA.mass
            * (right_tibia_to_robot * RobotMass::RIGHT_TIBIA.center).coords
        + RobotMass::RIGHT_ANKLE.mass
            * (right_ankle_to_robot * RobotMass::RIGHT_ANKLE.center).coords
        + RobotMass::RIGHT_FOOT.mass * (right_foot_to_robot * RobotMass::RIGHT_FOOT.center).coords)
        / RobotMass::TOTAL_MASS;

    //isometry based on imu
    let robot_to_parallel =
        Isometry3::rotation(Vector3::y() * robot_state.inertial_measurement_unit.angles.y)
            * Isometry3::rotation(Vector3::x() * robot_state.inertial_measurement_unit.angles.x);
    //center of mass rotated parallel to ground
    let center_of_mass_in_parallel = robot_to_parallel * Point::from(center_of_mass);
    //acceleration rotated parallel to ground
    let accelerometer_in_parallel = robot_to_parallel
        * point![
            robot_state.inertial_measurement_unit.accelerometer.x,
            robot_state.inertial_measurement_unit.accelerometer.y,
            robot_state.inertial_measurement_unit.accelerometer.z,
        ];

    //feet relative to robot position rotated parallel to ground
    let imu_adjusted_robot_to_left_sole =
        robot_to_parallel * Isometry3::from(left_sole_to_robot.translation.inverse());
    let imu_adjusted_robot_to_right_sole =
        robot_to_parallel * Isometry3::from(right_sole_to_robot.translation.inverse());

    //middle of feet regardless of z position becomes the ground coordinate
    let left_sole_to_right_sole =
        right_sole_to_robot.translation.vector - left_sole_to_robot.translation.vector;
    let left_sole_to_ground =
        0.5 * vector![left_sole_to_right_sole.x, left_sole_to_right_sole.y, 0.0];

    //robot to ground assuming moving ground which is halfway between left and right soles
    let robot_to_ground = if left_has_more_pressure {
        Translation::from(-left_sole_to_ground) * imu_adjusted_robot_to_left_sole
    } else {
        Translation::from(left_sole_to_ground) * imu_adjusted_robot_to_right_sole
    };

    //calculate position of soles relative to robot in parallel to ground coordinates
    let left_sole = robot_to_parallel * left_sole_to_robot * Point::origin();
    let right_sole = robot_to_parallel * right_sole_to_robot * Point::origin();

    let left_sole_to_robot_in_parallel = robot_to_parallel * left_sole_to_robot;
    let right_sole_to_robot_in_parallel = robot_to_parallel * right_sole_to_robot;
    // debug!(
    //     "Point in parallel{}",
    //     left_sole_to_robot_in_parallel * point![0.100103, -0.001572, 0.0]
    // );
    // // debug!(
    // //     "Vector in parallel{}",
    // //     left_sole_to_robot_in_parallel * vector![0.100103, -0.001572, 0.0]
    // // );
    // debug!("Left sole{}", left_sole);
    // debug!(
    //     "Subtract points{}",
    //     left_sole_to_robot_in_parallel * point![0.100103, -0.001572, 0.0] - left_sole
    // );

    // debug!(
    //     "Subtracting the positions{}",
    //     left_sole - (left_sole_to_robot_in_parallel * vector![0.100103, -0.001572, 0.0])
    // );
    // let temp_front_convex_left_sole =
    //     ((left_sole_to_robot_in_parallel * point![0.100103, -0.001572, 0.0]) - left_sole).into();

    //convex hull points of left foot
    let left_convex_hull_points = [
        [-0.054657, 0.029814],
        [-0.05457, -0.015151],
        [-0.049908, -0.023019],
        [-0.04262, -0.030603],
        [-0.037661, -0.033714],
        [-0.03297, -0.034351],
        [0.0577, -0.038771],
        [0.057705, -0.038771],
        [0.063956, -0.038362],
        [0.07396, -0.03729],
        [0.079707, -0.035319],
        [0.084651, -0.033221],
        [0.087653, -0.031482],
        [0.09181, -0.027692],
        [0.094015, -0.024299],
        [0.096873, -0.018801],
        [0.099424, -0.010149],
        [0.100103, -0.001572],
        [0.098996, 0.008695],
        [0.097019, 0.016504],
        [0.094001, 0.02418],
        [0.090468, 0.02951],
        [0.08455, 0.036101],
        [0.0799, 0.039545],
        [0.07416, 0.042654],
        [0.065683, 0.046146],
        [0.057212, 0.047683],
        [0.049916, 0.048183],
        [-0.031242, 0.051719],
        [-0.03593, 0.049621],
        [-0.040999, 0.045959],
        [-0.045156, 0.042039],
        [-0.04905, 0.037599],
    ];
    let left_convex_hull = left_convex_hull_points
        .iter()
        .map(|point| {
            (left_sole_to_robot_in_parallel * point![point[0], point[1], 0.0]) - Point::origin()
        })
        .map(|point| point![point.x, point.y])
        .collect::<Vec<_>>();
    let right_convex_hull = left_convex_hull_points
        .iter()
        .map(|point| {
            (right_sole_to_robot_in_parallel * point![point[0], -point[1], 0.0]) - Point::origin()
        })
        .map(|point| point![point.x, point.y])
        .collect::<Vec<_>>();
    // debug!("points:{:?}", temp);
    //Convex hull calculations
    let mut union_convex_hull = left_convex_hull.clone();
    let mut right_convex_hull_clone = right_convex_hull.clone();
    union_convex_hull.append(&mut right_convex_hull_clone);
    let convex_hull_2d_points = union_convex_hull
        .iter()
        .map(|point| ncollide2d::na::Point2::new(point.x, point.y))
        .collect::<Vec<_>>();
    let convex_hull_2d = ncollide2d::transformation::convex_hull(&convex_hull_2d_points)
        .unwrap()
        .0
        .iter()
        .map(|point| point![point[0], point[1]])
        .collect::<Vec<_>>(); //known issue here as last frame is passed with no data, needs to be fixed in while loop

    let left_hull_index = left_convex_hull
        .iter()
        .map(|point| convex_hull_2d.contains(point))
        .collect::<Vec<_>>();
    let right_hull_index = right_convex_hull
        .iter()
        .map(|point| convex_hull_2d.contains(point))
        .collect::<Vec<_>>();

    //Zero Moment Point

    let left_contact_point = left_convex_hull_points
        .iter()
        .map(|point| {
            (left_sole_to_robot_in_parallel * point![point[0], point[1], 0.0]) - Point::origin()
        })
        .map(|point| point.z)
        .min_by(|x, y| x.abs().partial_cmp(&y.abs()).unwrap())
        .unwrap();
    let right_contact_point = left_convex_hull_points
        .iter()
        .map(|point| {
            (right_sole_to_robot_in_parallel * point![point[0], -point[1], 0.0]) - Point::origin()
        })
        .map(|point| point.z)
        .min_by(|x, y| x.abs().partial_cmp(&y.abs()).unwrap())
        .unwrap();

    let z_foot_to_parallel = if left_has_more_pressure {
        -left_contact_point
    } else {
        -right_contact_point
    };
    let z = center_of_mass_in_parallel.z - z_foot_to_parallel;
    let x_com = center_of_mass_in_parallel.x;
    let y_com = center_of_mass_in_parallel.y;
    let x_hat = accelerometer_in_parallel.x;
    let y_hat = accelerometer_in_parallel.y;
    let g = 9.80665;

    let x_zero_moment_point_in_parallel = ((x_com * g) + (x_hat * z)) / g;
    let y_zero_moment_point_in_parallel = ((y_com * g) + (y_hat * z)) / g;

    AllTheCalculationsFunctionResult {
        center_of_mass: Point::from(center_of_mass),
        center_of_mass_in_parallel: Point::from(center_of_mass_in_parallel),
        accelerometer_in_parallel: Point::from(accelerometer_in_parallel),
        left_is_support_imu: left_sole.z <= right_sole.z,
        left_sole,
        right_sole,
        center_of_mass_in_ground: robot_to_ground * center_of_mass_in_parallel,
        left_convex_hull,
        right_convex_hull,
        left_hull_index,
        right_hull_index,
        x_zero_moment_point_in_parallel,
        y_zero_moment_point_in_parallel,
    }
}
