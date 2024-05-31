use std::{
    collections::BTreeMap,
    time::{Duration, SystemTime},
};

use color_eyre::Result;
use nalgebra::{matrix, Matrix2, Matrix2x4, Matrix4, Matrix4x2};
use serde::{Deserialize, Serialize};

use context_attribute::context;
use coordinate_systems::{Ground, Pixel, Robot};
use filtering::kalman_filter::KalmanFilter;
use framework::{AdditionalOutput, HistoricInput, MainOutput, PerceptionInput};
use geometry::circle::Circle;
use linear_algebra::{vector, Isometry3, Orientation3, Point2, Vector2};
use projection::{camera_matrices::CameraMatrices, camera_matrix::CameraMatrix, Projection};
use types::{
    ball::Ball,
    ball_filter::Hypothesis,
    ball_position::{BallPosition, HypotheticalBallPosition},
    cycle_time::CycleTime,
    field_dimensions::FieldDimensions,
    limb::{is_above_limbs, Limb, ProjectedLimbs},
    multivariate_normal_distribution::MultivariateNormalDistribution,
    parameters::{BallFilterParameters, CameraMatrixParameters},
    robot_dimensions::RobotDimensions,
    robot_kinematics::RobotKinematics,
    sensor_data::SensorData,
    support_foot::Side,
};

use crate::camera_matrix_calculator::head_to_camera;

#[derive(Deserialize, Serialize, Debug)]
struct SyncData {
    imu: Vector2<Robot>,
    robot_kinematics: RobotKinematics,
    support_side: Side,
}

#[derive(Deserialize, Serialize, Default)]
pub struct BallFilterSync {
    hypotheses: Vec<Hypothesis>,

    sync_data: BTreeMap<SystemTime, SyncData>,
    ball_data: BTreeMap<SystemTime, Vec<Ball>>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ball_filter_hypotheses: AdditionalOutput<Vec<Hypothesis>, "ball_filter_hypotheses">,
    best_ball_hypothesis: AdditionalOutput<Option<Hypothesis>, "best_ball_hypothesis">,
    best_ball_state: AdditionalOutput<Option<MultivariateNormalDistribution<4>>, "best_ball_state">,
    chooses_resting_model: AdditionalOutput<bool, "chooses_resting_model">,
    filtered_balls_in_image_bottom:
        AdditionalOutput<Vec<Circle<Pixel>>, "filtered_balls_in_image_bottom">,
    filtered_balls_in_image_top:
        AdditionalOutput<Vec<Circle<Pixel>>, "filtered_balls_in_image_top">,

    current_odometry_to_last_odometry:
        HistoricInput<Option<nalgebra::Isometry2<f32>>, "current_odometry_to_last_odometry?">,
    historic_camera_matrices: HistoricInput<Option<CameraMatrices>, "camera_matrices?">,

    camera_matrices: RequiredInput<Option<CameraMatrices>, "camera_matrices?">,
    cycle_time: Input<CycleTime, "cycle_time">,
    sensor_data: Input<SensorData, "sensor_data">,
    support_side: RequiredInput<Option<Side>, "support_foot.support_side?">,
    robot_kinematics: Input<RobotKinematics, "robot_kinematics">,

    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    top_camera_matrix_parameters:
        Parameter<CameraMatrixParameters, "camera_matrix_parameters.vision_top">,
    ball_filter_configuration: Parameter<BallFilterParameters, "ball_filter">,
    imu_delay: Parameter<f32, "imu_delay">,

    balls_bottom: PerceptionInput<Option<Vec<Ball>>, "VisionBottom", "balls?">,
    balls_top: PerceptionInput<Option<Vec<Ball>>, "VisionTop", "balls?">,
    projected_limbs: PerceptionInput<Option<ProjectedLimbs>, "VisionBottom", "projected_limbs?">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub ball_position_sync: MainOutput<Option<BallPosition<Ground>>>,
    pub removed_ball_positions_sync: MainOutput<Vec<Point2<Ground>>>,
    pub hypothetical_ball_positions_sync: MainOutput<Vec<HypotheticalBallPosition<Ground>>>,
    pub balls_top_sync: MainOutput<Vec<(SystemTime, Vec<Ball>)>>,
    pub balls_bottom_sync: MainOutput<Vec<(SystemTime, Vec<Ball>)>>,
}

impl BallFilterSync {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self::default())
    }

    fn persistent_balls_in_control_cycle(
        &mut self,
        context: &mut CycleContext,
    ) -> Vec<(SystemTime, Vec<Ball>)> {
        context
            .balls_top
            .persistent
            .iter()
            // .zip(context.balls_bottom.persistent.values())
            .map(|(detection_time, balls_top)| {
                let balls = balls_top
                    .iter()
                    // .chain(balls_bottom.iter())
                    .filter_map(|data| data.as_ref())
                    .flat_map(|data| data.iter().cloned())
                    .collect();
                (*detection_time, balls)
            })
            .collect()
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        self.sync_data.insert(
            context.cycle_time.start_time,
            SyncData {
                imu: context.sensor_data.inertial_measurement_unit.roll_pitch,
                robot_kinematics: context.robot_kinematics.clone(),
                support_side: *context.support_side,
            },
        );
        for (time, balls) in self.persistent_balls_in_control_cycle(&mut context) {
            self.ball_data.insert(time, balls);
        }

        let mut persistent_updates = Vec::new();

        let imu_time = context.cycle_time.start_time - Duration::from_secs_f32(*context.imu_delay);
        println!("-------------");
        for (ball_time, balls) in &self.ball_data {
            if balls.is_empty() {
                persistent_updates.push((
                    context.cycle_time.start_time, // TODO: this is wrong
                    vec![],
                ));
                continue;
            }
            if *ball_time >= imu_time {
                continue;
            }

            dbg!(imu_time);
            let ball_times: Vec<_> = self.ball_data.keys().collect();
            dbg!(ball_times);
            let sync_times: Vec<_> = self.sync_data.keys().collect();
            dbg!(sync_times);
            let sync_data = find_latest(*ball_time, &self.sync_data).unwrap();
            let imu = context.sensor_data.inertial_measurement_unit.roll_pitch;

            let ground_to_robot =
                make_ground_to_robot(&sync_data.1.robot_kinematics, imu, sync_data.1.support_side);
            let camera_matrix = make_camera_matrix(
                ground_to_robot,
                context.robot_kinematics,
                *context.top_camera_matrix_parameters,
            );

            for ball in balls {
                let Ok(position) =
                    camera_matrix.pixel_to_ground_with_z(ball.image_location.center, 0.05)
                else {
                    continue;
                };
                persistent_updates.push((
                    context.cycle_time.start_time, // TODO: this is wrong
                    vec![Ball {
                        position,
                        image_location: ball.image_location,
                    }],
                ));
            }
        }

        self.ball_data.retain(|time, _| *time > imu_time);
        let ball_time = self
            .ball_data
            .first_key_value()
            .map(|v| *v.0)
            .unwrap_or(context.cycle_time.start_time)
            .min(imu_time)
            - Duration::from_secs_f32(0.1);
        self.sync_data.retain(|time, _| *time >= ball_time);

        let removed_hypotheses = self.advance_all_hypotheses(persistent_updates, &context);

        context
            .ball_filter_hypotheses
            .fill_if_subscribed(|| self.hypotheses.clone());
        let ball_radius = context.field_dimensions.ball_radius;

        let ball_positions = self
            .hypotheses
            .iter()
            .map(|hypothesis| hypothesis.selected_ball_position(context.ball_filter_configuration))
            .collect::<Vec<_>>();
        context.filtered_balls_in_image_top.fill_if_subscribed(|| {
            project_to_image(&ball_positions, &context.camera_matrices.top, ball_radius)
        });
        context
            .filtered_balls_in_image_bottom
            .fill_if_subscribed(|| {
                project_to_image(
                    &ball_positions,
                    &context.camera_matrices.bottom,
                    ball_radius,
                )
            });

        context.best_ball_hypothesis.fill_if_subscribed(|| {
            self.find_best_hypothesis(context.ball_filter_configuration)
                .cloned()
        });

        context.best_ball_state.fill_if_subscribed(|| {
            self.find_best_hypothesis(context.ball_filter_configuration)
                .map(|hypothesis| hypothesis.selected_state(context.ball_filter_configuration))
        });

        let ball_position = self
            .find_best_hypothesis(context.ball_filter_configuration)
            .map(|hypothesis| {
                context.chooses_resting_model.fill_if_subscribed(|| {
                    hypothesis.is_resting(context.ball_filter_configuration)
                });
                hypothesis.selected_ball_position(context.ball_filter_configuration)
            });
        let removed_ball_positions = removed_hypotheses
            .into_iter()
            .filter(|hypothesis| {
                hypothesis.validity >= context.ball_filter_configuration.validity_output_threshold
            })
            .map(|hypothesis| {
                hypothesis
                    .selected_ball_position(context.ball_filter_configuration)
                    .position
            })
            .collect::<Vec<_>>();
        let hypothetical_ball_positions = self
            .hypotheses
            .iter()
            .filter_map(|hypothesis| {
                if hypothesis.validity < context.ball_filter_configuration.validity_output_threshold
                {
                    Some(HypotheticalBallPosition {
                        position: hypothesis
                            .selected_ball_position(context.ball_filter_configuration)
                            .position,
                        validity: hypothesis.validity,
                    })
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();

        let balls_top: Vec<(SystemTime, Vec<Ball>)> = context
            .balls_top
            .persistent
            .iter()
            .map(|(detection_time, balls)| {
                let balls = balls
                    .iter()
                    .filter_map(|data| data.as_ref())
                    .flat_map(|data| data.iter())
                    .cloned()
                    .collect();
                (*detection_time, balls)
            })
            .collect();
        let balls_bottom: Vec<(SystemTime, Vec<Ball>)> = context
            .balls_bottom
            .persistent
            .iter()
            .map(|(detection_time, balls)| {
                let balls = balls
                    .iter()
                    .filter_map(|data| data.as_ref())
                    .flat_map(|data| data.iter())
                    .cloned()
                    .collect();
                (*detection_time, balls)
            })
            .collect();

        Ok(MainOutputs {
            ball_position_sync: ball_position.into(),
            removed_ball_positions_sync: removed_ball_positions.into(),
            hypothetical_ball_positions_sync: hypothetical_ball_positions.into(),
            balls_top_sync: balls_top.into(),
            balls_bottom_sync: balls_bottom.into(),
        })
    }

    fn advance_all_hypotheses(
        &mut self,
        measurements: Vec<(SystemTime, Vec<Ball>)>,
        context: &CycleContext,
    ) -> Vec<Hypothesis> {
        for (detection_time, balls) in measurements {
            let current_odometry_to_last_odometry = context
                .current_odometry_to_last_odometry
                .get(&detection_time)
                .expect("current_odometry_to_last_odometry should not be None");
            println!("eeeeee");
            // dbg!(&self.hypotheses);
            self.predict_hypotheses_with_odometry(
                context.ball_filter_configuration.velocity_decay_factor,
                current_odometry_to_last_odometry.inverse(),
                Matrix4::from_diagonal(&context.ball_filter_configuration.process_noise),
            );
            println!("oooo");
            // dbg!(&self.hypotheses);

            let camera_matrices = context.historic_camera_matrices.get(&detection_time);
            let projected_limbs_bottom = context
                .projected_limbs
                .persistent
                .get(&detection_time)
                .and_then(|limbs| limbs.last())
                .and_then(|limbs| *limbs);
            self.decay_hypotheses(
                camera_matrices,
                projected_limbs_bottom,
                context.field_dimensions.ball_radius,
                context.ball_filter_configuration,
            );

            for ball in balls {
                self.update_hypotheses_with_measurement(
                    ball.position,
                    detection_time,
                    context.ball_filter_configuration,
                );
            }
        }

        self.remove_hypotheses(
            context.cycle_time.start_time,
            context.ball_filter_configuration,
            context.field_dimensions,
        )
    }

    fn decay_hypotheses(
        &mut self,
        camera_matrices: Option<&CameraMatrices>,
        projected_limbs: Option<&ProjectedLimbs>,
        ball_radius: f32,
        configuration: &BallFilterParameters,
    ) {
        for hypothesis in self.hypotheses.iter_mut() {
            let ball_in_view = match (camera_matrices.as_ref(), projected_limbs.as_ref()) {
                (Some(camera_matrices), Some(projected_limbs)) => {
                    is_visible_to_camera(
                        hypothesis,
                        &camera_matrices.bottom,
                        ball_radius,
                        &projected_limbs.limbs,
                        configuration,
                    ) || is_visible_to_camera(
                        hypothesis,
                        &camera_matrices.top,
                        ball_radius,
                        &[],
                        configuration,
                    )
                }
                _ => false,
            };

            let decay_factor = if ball_in_view {
                configuration.visible_validity_exponential_decay_factor
            } else {
                configuration.hidden_validity_exponential_decay_factor
            };
            hypothesis.validity *= decay_factor;
        }
    }

    fn predict_hypotheses_with_odometry(
        &mut self,
        velocity_decay_factor: f32,
        last_odometry_to_current_odometry: nalgebra::Isometry2<f32>,
        process_noise: Matrix4<f32>,
    ) {
        for hypothesis in self.hypotheses.iter_mut() {
            let cycle_time = 0.012;
            let constant_velocity_prediction = matrix![
                1.0, 0.0, cycle_time, 0.0;
                0.0, 1.0, 0.0, cycle_time;
                0.0, 0.0, velocity_decay_factor, 0.0;
                0.0, 0.0, 0.0, velocity_decay_factor;
            ];
            let rotation = last_odometry_to_current_odometry
                .rotation
                .to_rotation_matrix();
            let state_rotation = matrix![
                rotation[(0, 0)], rotation[(0, 1)], 0.0, 0.0;
                rotation[(1, 0)], rotation[(1, 1)], 0.0, 0.0;
                0.0, 0.0, rotation[(0, 0)], rotation[(0, 1)];
                0.0, 0.0, rotation[(1, 0)], rotation[(1, 1)];
            ];
            let state_prediction = constant_velocity_prediction * state_rotation;
            let control_input_model = Matrix4x2::identity();
            let odometry_translation = last_odometry_to_current_odometry.translation.vector;
            hypothesis.moving_state.predict(
                state_prediction,
                control_input_model,
                odometry_translation,
                process_noise,
            );
            hypothesis.resting_state.predict(
                state_prediction,
                control_input_model,
                odometry_translation,
                process_noise,
            );
        }
    }

    fn update_hypothesis_with_measurement(
        hypothesis: &mut Hypothesis,
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        configuration: &BallFilterParameters,
    ) {
        hypothesis.moving_state.update(
            Matrix2x4::identity(),
            detected_position.inner.coords,
            Matrix2::from_diagonal(&configuration.measurement_noise_moving)
                * detected_position.coords().norm_squared(),
        );
        hypothesis.resting_state.update(
            Matrix2x4::identity(),
            detected_position.inner.coords,
            Matrix2::from_diagonal(&configuration.measurement_noise_resting)
                * detected_position.coords().norm_squared(),
        );

        if !hypothesis.is_resting(configuration) {
            hypothesis.resting_state.mean = hypothesis.moving_state.mean;
        }
        hypothesis.last_update = detection_time;
        hypothesis.validity += 1.0;
    }

    fn update_hypotheses_with_measurement(
        &mut self,
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        configuration: &BallFilterParameters,
    ) {
        let mut matching_hypotheses = self
            .hypotheses
            .iter_mut()
            .filter(|hypothesis| {
                (hypothesis.moving_state.mean.xy() - detected_position.inner.coords).norm()
                    < configuration.measurement_matching_distance
                    || (hypothesis.resting_state.mean.xy() - detected_position.inner.coords).norm()
                        < configuration.measurement_matching_distance
            })
            .peekable();

        if matching_hypotheses.peek().is_none() {
            self.spawn_hypothesis(detected_position, detection_time, configuration);
            return;
        }
        matching_hypotheses.for_each(|hypothesis| {
            Self::update_hypothesis_with_measurement(
                hypothesis,
                detected_position,
                detection_time,
                configuration,
            )
        });
    }

    fn find_best_hypothesis(&self, configuration: &BallFilterParameters) -> Option<&Hypothesis> {
        self.hypotheses
            .iter()
            .filter(|hypothesis| hypothesis.validity > configuration.validity_output_threshold)
            .max_by(|left, right| left.validity.total_cmp(&right.validity))
    }

    fn spawn_hypothesis(
        &mut self,
        detected_position: Point2<Ground>,
        detection_time: SystemTime,
        configuration: &BallFilterParameters,
    ) {
        let initial_state =
            nalgebra::vector![detected_position.x(), detected_position.y(), 0.0, 0.0];
        let new_hypothesis = Hypothesis {
            moving_state: MultivariateNormalDistribution {
                mean: initial_state,
                covariance: Matrix4::from_diagonal(&configuration.initial_covariance),
            },
            resting_state: MultivariateNormalDistribution {
                mean: initial_state,
                covariance: Matrix4::from_diagonal(&configuration.initial_covariance),
            },
            validity: 1.0,
            last_update: detection_time,
        };
        self.hypotheses.push(new_hypothesis);
    }

    fn remove_hypotheses(
        &mut self,
        now: SystemTime,
        configuration: &BallFilterParameters,
        field_dimensions: &FieldDimensions,
    ) -> Vec<Hypothesis> {
        let (retained_hypotheses, removed_hypotheses) = self
            .hypotheses
            .drain(..)
            .partition::<Vec<_>, _>(|hypothesis| {
                let selected_position = hypothesis.selected_ball_position(configuration).position;
                let is_inside_field = {
                    selected_position.coords().x().abs()
                        < field_dimensions.length / 2.0 + field_dimensions.border_strip_width
                        && selected_position.y().abs()
                            < field_dimensions.width / 2.0 + field_dimensions.border_strip_width
                };
                now.duration_since(hypothesis.last_update)
                    .expect("Time has run backwards")
                    < configuration.hypothesis_timeout
                    && hypothesis.validity > configuration.validity_discard_threshold
                    && is_inside_field
            });
        let mut deduplicated_hypotheses = Vec::<Hypothesis>::new();
        for hypothesis in retained_hypotheses {
            let hypothesis_in_merge_distance =
                deduplicated_hypotheses
                    .iter_mut()
                    .find(|existing_hypothesis| {
                        (existing_hypothesis
                            .selected_ball_position(configuration)
                            .position
                            - hypothesis.selected_ball_position(configuration).position)
                            .norm()
                            < configuration.hypothesis_merge_distance
                    });
            match hypothesis_in_merge_distance {
                Some(existing_hypothesis) => {
                    let update_state = hypothesis.selected_state(configuration);
                    existing_hypothesis.moving_state.update(
                        Matrix4::identity(),
                        update_state.mean,
                        update_state.covariance,
                    );
                    existing_hypothesis.resting_state.update(
                        Matrix4::identity(),
                        update_state.mean,
                        update_state.covariance,
                    );
                }
                None => deduplicated_hypotheses.push(hypothesis),
            }
        }
        self.hypotheses = deduplicated_hypotheses;
        removed_hypotheses
    }
}

fn project_to_image(
    ball_position: &[BallPosition<Ground>],
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
) -> Vec<Circle<Pixel>> {
    ball_position
        .iter()
        .filter_map(|ball_position| {
            let position_in_image = camera_matrix
                .ground_with_z_to_pixel(ball_position.position, ball_radius)
                .ok()?;
            let radius = camera_matrix
                .get_pixel_radius(ball_radius, position_in_image)
                .ok()?;
            Some(Circle {
                center: position_in_image,
                radius,
            })
        })
        .collect()
}

fn is_visible_to_camera(
    hypothesis: &Hypothesis,
    camera_matrix: &CameraMatrix,
    ball_radius: f32,
    projected_limbs: &[Limb],
    configuration: &BallFilterParameters,
) -> bool {
    let position_on_ground = hypothesis.selected_ball_position(configuration).position;
    let position_in_image =
        match camera_matrix.ground_with_z_to_pixel(position_on_ground, ball_radius) {
            Ok(position_in_image) => position_in_image,
            Err(_) => return false,
        };
    (0.0..640.0).contains(&position_in_image.x())
        && (0.0..480.0).contains(&position_in_image.y())
        && is_above_limbs(position_in_image, projected_limbs)
}

fn find_latest<T>(time: SystemTime, map: &BTreeMap<SystemTime, T>) -> Option<(&SystemTime, &T)> {
    map.iter().rev().find(|item| *item.0 <= time)
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
