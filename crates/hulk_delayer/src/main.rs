#![recursion_limit = "256"]
use core::f32;
use std::time::SystemTime;
use std::{collections::BTreeMap, time::Duration};
use std::{env::args, sync::Arc};

use color_eyre::{
    eyre::{Result, WrapErr},
    install,
};

use control::camera_matrix_calculator::head_to_camera;
use coordinate_systems::{Ground, LeftAnkle, Robot};
use hardware::{
    ActuatorInterface, CameraInterface, IdInterface, NetworkInterface, PathsInterface,
    RecordingInterface, SensorInterface, SpeakerInterface, TimeInterface,
};
use linear_algebra::{vector, Isometry3, Orientation3, Vector2};
use projection::{camera_matrix::CameraMatrix, Projection};
use types::{
    audio::SpeakerRequest,
    ball::Ball,
    camera_position::CameraPosition,
    hardware::{Ids, Paths},
    joints::Joints,
    led::Leds,
    messages::{IncomingMessage, OutgoingMessage},
    parameters::CameraMatrixParameters,
    robot_dimensions::RobotDimensions,
    robot_kinematics::RobotKinematics,
    sensor_data::SensorData,
    support_foot::Side,
    ycbcr422_image::YCbCr422Image,
};

use crate::execution::Replayer;

pub trait HardwareInterface:
    ActuatorInterface
    + IdInterface
    + NetworkInterface
    + PathsInterface
    + RecordingInterface
    + SensorInterface
    + SpeakerInterface
    + TimeInterface
{
}

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));
const INSTANCE_NAME: &str = "Control";

struct ImageExtractorHardwareInterface {
    ids: Ids,
}

impl CameraInterface for ImageExtractorHardwareInterface {
    fn read_from_camera(&self, _camera_position: CameraPosition) -> Result<YCbCr422Image> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl PathsInterface for ImageExtractorHardwareInterface {
    fn get_paths(&self) -> Paths {
        Paths {
            motions: "etc/motions".into(),
            neural_networks: "etc/neural_networks".into(),
            sounds: "etc/sounds".into(),
        }
    }
}

impl TimeInterface for ImageExtractorHardwareInterface {
    fn get_now(&self) -> SystemTime {
        SystemTime::now()
    }
}

impl HardwareInterface for ImageExtractorHardwareInterface {}

impl ActuatorInterface for ImageExtractorHardwareInterface {
    fn write_to_actuators(
        &self,
        _positions: Joints<f32>,
        _stiffnesses: Joints<f32>,
        _leds: Leds,
    ) -> Result<()> {
        Ok(())
    }
}

impl IdInterface for ImageExtractorHardwareInterface {
    fn get_ids(&self) -> Ids {
        self.ids.clone()
    }
}

impl NetworkInterface for ImageExtractorHardwareInterface {
    fn read_from_network(&self) -> Result<IncomingMessage> {
        panic!("Replayer cannot produce data from hardware")
    }

    fn write_to_network(&self, _message: OutgoingMessage) -> Result<()> {
        Ok(())
    }
}

impl RecordingInterface for ImageExtractorHardwareInterface {
    fn should_record(&self) -> bool {
        false
    }

    fn set_whether_to_record(&self, _enable: bool) {}
}

impl SensorInterface for ImageExtractorHardwareInterface {
    fn read_from_sensors(&self) -> Result<SensorData> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl SpeakerInterface for ImageExtractorHardwareInterface {
    fn write_to_speakers(&self, _request: SpeakerRequest) {}
}

fn main() -> Result<()> {
    install()?;

    let replay_path = args()
        .nth(1)
        .expect("expected replay path as first parameter");

    let parameters_directory = args().nth(3).unwrap_or(replay_path.clone());
    let id = "replayer".to_string();

    let mut replayer = Replayer::new(
        Arc::new(ImageExtractorHardwareInterface {
            ids: Ids {
                body_id: "Delayer".to_string(),
                head_id: "Delayer".to_string(),
            },
        }),
        parameters_directory,
        id.clone(),
        id,
        replay_path,
    )
    .wrap_err("failed to create image extractor")?;

    let unknown_indices_error_message =
        format!("could not find recording indices for `{INSTANCE_NAME}`");
    let timings: Vec<_> = replayer
        .get_recording_indices()
        .get(INSTANCE_NAME)
        .expect(&unknown_indices_error_message)
        .iter()
        .collect();

    let mut imu = BTreeMap::<SystemTime, Vector2<Robot>>::new();
    let mut balls_top = Vec::<Ball>::new();

    for timing in &timings {
        let frame = replayer
            .get_recording_indices_mut()
            .get_mut(INSTANCE_NAME)
            .map(|index| {
                index
                    .find_latest_frame_up_to(timing.timestamp)
                    .expect("failed to find latest frame")
            })
            .expect(&unknown_indices_error_message);

        if let Some(frame) = frame {
            let reader = replayer.control_reader();
            replayer
                .replay(INSTANCE_NAME, frame.timing.timestamp, &frame.data)
                .expect("failed to replay frame");

            let database = reader.next();
            imu.insert(
                frame.timing.timestamp,
                database
                    .main_outputs
                    .sensor_data
                    .inertial_measurement_unit
                    .roll_pitch,
            );
            for (_time, items) in &database.main_outputs.balls_top {
                balls_top.extend(items.clone());
            }
        }
    }

    analyze(&balls_top);
    println!("now shifting");

    let delta = 0.012;
    let best = (-10..=10)
        .map(|i| {
            let delay = delta * i as f32;
            let balls_top = balls_from_frames(&mut replayer, &imu, delay);
            println!("{delay}");
            (i, analyze(&balls_top))
        })
        .min_by_key(|(_, metric)| ordered_float::NotNan::new(*metric).unwrap());

    println!("{best:#?}");

    Ok(())
}

fn sample(values: &BTreeMap<SystemTime, Vector2<Robot>>, time: SystemTime) -> Vector2<Robot> {
    let start = time - Duration::from_secs(1);
    let end = time + Duration::from_secs(1);
    let relevant_area: Vec<_> = values.range(start..end).to_owned().collect();
    for index in 0..relevant_area.len() - 1 {
        if *relevant_area[index].0 <= time && time < *relevant_area[index + 1].0 {
            let a = relevant_area[index];
            let b = relevant_area[index + 1];

            let t = time.duration_since(*a.0).unwrap().as_secs_f32();
            let segment_duration = b.0.duration_since(*a.0).unwrap().as_secs_f32();

            let f = t / segment_duration;

            return *a.1;
            // return *a.1 + (*b.1 - *a.1) * f;
        }
    }

    if time < *relevant_area.first().unwrap().0 {
        return *relevant_area.first().unwrap().1;
    }
    if time >= *relevant_area.last().unwrap().0 {
        return *relevant_area.last().unwrap().1;
    }
    println!("Relevant: {:#?}", &relevant_area[..2]);
    println!("First: {:#?}", values.keys().next().unwrap());
    println!("Current: {:#?}", time);

    unreachable!()
}

fn balls_from_frames(
    replayer: &mut Replayer<ImageExtractorHardwareInterface>,
    all_imu: &BTreeMap<SystemTime, Vector2<Robot>>,
    delay: f32,
) -> Vec<Ball> {
    let mut balls_top = Vec::<Ball>::new();
    let unknown_indices_error_message =
        format!("could not find recording indices for `{INSTANCE_NAME}`");
    let timings: Vec<_> = replayer
        .get_recording_indices()
        .get(INSTANCE_NAME)
        .expect(&unknown_indices_error_message)
        .iter()
        .collect();

    for timing in timings {
        let frame = replayer
            .get_recording_indices_mut()
            .get_mut(INSTANCE_NAME)
            .map(|index| {
                index
                    .find_latest_frame_up_to(timing.timestamp)
                    .expect("failed to find latest frame")
            })
            .expect(&unknown_indices_error_message);

        if let Some(frame) = frame {
            let reader = replayer.control_reader();
            replayer
                .replay(INSTANCE_NAME, frame.timing.timestamp, &frame.data)
                .expect("failed to replay frame");

            let database = reader.next();
            let Some(support_side) = database.main_outputs.support_foot.support_side else {
                continue;
            };
            let sample_time = if delay > 0.0 {
                timing.timestamp + Duration::from_secs_f32(delay)
            } else {
                timing.timestamp - Duration::from_secs_f32(-delay)
            };
            let imu = sample(&all_imu, sample_time);
            let ground_to_robot =
                make_ground_to_robot(&database.main_outputs.robot_kinematics, imu, support_side);
            let camera_matrix = make_camera_matrix(
                ground_to_robot,
                &database.main_outputs.robot_kinematics,
                CameraMatrixParameters {
                    camera_pitch: -1.2,
                    extrinsic_rotations: nalgebra::vector![0.0, 0.0, 0.0],
                    focal_lengths: nalgebra::vector![0.95, 1.27],
                    cc_optical_center: nalgebra::point![0.5, 0.5],
                },
            );

            for (_time, items) in &database.main_outputs.balls_top {
                balls_top.extend(items.iter().map(|ball| {
                    Ball {
                        position: camera_matrix
                            .pixel_to_ground(ball.image_location.center)
                            .unwrap(),
                        image_location: ball.image_location,
                    }
                }));
            }
        }
    }

    balls_top
}

fn analyze(balls: &[Ball]) -> f32 {
    let mut min: Vector2<Ground> = vector![f32::INFINITY, f32::INFINITY];
    let mut max: Vector2<Ground> = vector![f32::NEG_INFINITY, f32::NEG_INFINITY];
    let mut avg: Vector2<Ground> = vector![0.0, 0.0];
    for ball in balls {
        let position = ball.position;
        min.inner.x = min.x().min(position.x());
        min.inner.y = min.y().min(position.y());
        max.inner.x = max.x().max(position.x());
        max.inner.y = max.y().max(position.y());
        avg += position.coords();
    }
    avg /= balls.len() as f32;
    let mut variance = 0.0;
    for ball in balls {
        variance += (ball.position.coords() - avg).norm_squared();
    }
    variance /= balls.len() as f32;
    let stddev = variance.sqrt();

    println!("area: {:?}", (max - min).inner);
    println!("avg: {:?}", avg.inner);
    println!("variance: {:?}", variance);
    println!("stddev: {:?}", stddev);
    stddev
}

fn make_ground_to_robot(
    robot_kinematics: &RobotKinematics,
    imu: Vector2<Robot>,
    support_side: Side,
) -> Isometry3<Ground, Robot> {
    struct LeftSoleHorizontal;
    struct RightSoleHorizontal;

    let imu_orientation = Orientation3::from_euler_angles(imu.x(), imu.y(), 0.0).mirror();

    let left_sole_horizontal_to_robot = Isometry3::from_parts(
        robot_kinematics
            .left_leg
            .sole_to_robot
            .translation()
            .coords(),
        imu_orientation,
    );
    let right_sole_horizontal_to_robot = Isometry3::from_parts(
        robot_kinematics
            .right_leg
            .sole_to_robot
            .translation()
            .coords(),
        imu_orientation,
    );

    let left_sole_in_robot = robot_kinematics.left_leg.sole_to_robot.translation();
    let right_sole_in_robot = robot_kinematics.right_leg.sole_to_robot.translation();

    let left_sole_to_right_sole = right_sole_in_robot - left_sole_in_robot;
    let ground_to_left_sole = Isometry3::<Ground, LeftSoleHorizontal>::from(
        vector![
            left_sole_to_right_sole.x(),
            left_sole_to_right_sole.y(),
            0.0
        ] / 2.0,
    );
    let ground_to_right_sole = Isometry3::<Ground, RightSoleHorizontal>::from(
        -vector![
            left_sole_to_right_sole.x(),
            left_sole_to_right_sole.y(),
            0.0
        ] / 2.0,
    );

    match support_side {
        Side::Left => left_sole_horizontal_to_robot * ground_to_left_sole,
        Side::Right => right_sole_horizontal_to_robot * ground_to_right_sole, //ground_to_right_sole * robot_to_right_support_sole,
    }
}

fn make_camera_matrix(
    ground_to_robot: Isometry3<Ground, Robot>,
    robot_kinematics: &RobotKinematics,
    top_camera_matrix_parameters: CameraMatrixParameters,
) -> CameraMatrix {
    let image_size = vector![640.0, 480.0];
    let head_to_top_camera = head_to_camera(
        top_camera_matrix_parameters.extrinsic_rotations,
        top_camera_matrix_parameters.camera_pitch.to_radians(),
        RobotDimensions::HEAD_TO_TOP_CAMERA,
    );
    CameraMatrix::from_normalized_focal_and_center(
        top_camera_matrix_parameters.focal_lengths,
        top_camera_matrix_parameters.cc_optical_center,
        image_size,
        ground_to_robot,
        robot_kinematics.head.head_to_robot.inverse(),
        head_to_top_camera,
    )
}
