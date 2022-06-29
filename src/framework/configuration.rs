use std::ops::Range;
use std::{path::PathBuf, time::Duration};

use macros::SerializeHierarchy;
use nalgebra::{Matrix3, Point2, Point3, Vector2, Vector3, Vector4};
use serde::{Deserialize, Serialize};
use spl_network::PlayerNumber;

use crate::types::{
    ArmJoints, FieldDimensions, HeadJoints, InitialPose, Joints, KickStep, MotionCommand, Players,
    Role, Step,
};

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Configuration {
    pub field_dimensions: FieldDimensions,
    pub audio: Audio,
    pub control: Control,
    #[leaf]
    pub player_number: PlayerNumber,
    pub spl_network: SplNetwork,
    pub vision_top: Vision,
    pub vision_bottom: Vision,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Audio {
    pub whistle_detection: WhistleDetection,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct WhistleDetection {
    pub detection_band: Range<f32>,
    pub background_noise_scaling: f32,
    pub whistle_scaling: f32,
    pub number_of_chunks: usize,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Control {
    pub ball_filter: BallFilter,
    pub behavior: Behavior,
    pub center_head_position: HeadJoints,
    pub fall_protection: FallProtection,
    pub fall_state_estimation: FallStateEstimation,
    pub game_state_filter: GameStateFilter,
    pub ground_contact_detector: HighDetector,
    pub head_motion_limits: HeadMotionLimits,
    pub dispatching_head_interpolator: DispatchingHeadInterpolator,
    pub head_motion: HeadMotion,
    pub localization: Localization,
    pub look_at: LookAt,
    pub look_around: LookAround,
    pub orientation_filter: OrientationFilter,
    pub penalized_pose: Joints,
    pub ready_pose: Joints,
    pub role_assignment: RoleAssignment,
    pub step_planner: StepPlanner,
    pub stand_up: StandUp,
    pub support_foot_estimation: SupportFootEstimation,
    pub walking_engine: WalkingEngine,
    pub whistle_filter: WhistleFilter,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Localization {
    pub circle_measurement_noise: Vector2<f32>,
    pub gradient_convergence_threshold: f32,
    pub gradient_descent_step_size: f32,
    pub hypothesis_prediction_score_reduction_factor: f32,
    pub hypothesis_retain_factor: f32,
    pub initial_hypothesis_covariance: Matrix3<f32>,
    pub initial_hypothesis_score: f32,
    pub initial_poses: Players<InitialPose>,
    pub line_length_acceptance_factor: f32,
    pub line_measurement_noise: Vector2<f32>,
    pub maximum_amount_of_gradient_descent_iterations: usize,
    pub maximum_amount_of_outer_iterations: usize,
    pub minimum_fit_error: f32,
    pub odometry_noise: Vector3<f32>,
    pub use_line_measurements: bool,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct StepPlanner {
    pub injected_step: Option<Step>,
    pub max_step_size: Step,
    pub max_step_size_backwards: f32,
    pub translation_exponent: f32,
    pub rotation_exponent: f32,
    pub inside_turn_ratio: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct SupportFootEstimation {
    pub hysteresis: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct HighDetector {
    pub pressure_threshold: f32,
    pub hysteresis: f32,
    pub timeout: Duration,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct RoleAssignment {
    #[leaf]
    pub forced_role: Option<Role>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Behavior {
    #[leaf]
    pub injected_motion_command: Option<MotionCommand>,
    pub role_positions: RolePositions,
    pub dribble_pose: DribblePose,
    pub walk_to_pose: WalkToPose,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct RolePositions {
    pub defender_aggressive_ring_radius: f32,
    pub defender_passive_ring_radius: f32,
    pub defender_y_offset: f32,
    pub striker_supporter_minimum_x: f32,
    pub keeper_x_offset: f32,
    pub striker_set_position: Vector2<f32>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct DribblePose {
    pub offset: Vector2<f32>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct WalkToPose {
    pub target_reached_threshold: Vector2<f32>,
    pub robot_radius: f32,
    pub hybrid_align_distance: f32,
    pub distance_to_be_aligned: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct GameStateFilter {
    pub max_wait_for_ready_message: f32,
    pub whistle_acceptance_goal_distance: Vector2<f32>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct WalkingEngine {
    pub walk_hip_height: f32,
    pub torso_offset: f32,
    pub minimal_step_duration: Duration,
    pub shoulder_pitch_factor: f32,
    pub base_foot_lift: f32,
    pub base_step_duration: Duration,
    pub starting_step_duration: Duration,
    pub starting_step_foot_lift: f32,
    pub gyro_balance_factor: f32,
    pub swing_foot_pitch_error_leveling_factor: f32,
    pub swing_foot_backwards_imu_leveling_factor: f32,
    pub max_level_adjustment_velocity: f32,
    pub max_forward_acceleration: f32,
    pub forward_foot_support_offset: f32,
    pub backward_foot_support_offset: f32,
    pub max_step_adjustment: f32,
    pub step_duration_increase: Step,
    pub leg_stiffness_stand: f32,
    pub leg_stiffness_walk: f32,
    pub arm_stiffness: f32,
    pub gyro_low_pass_factor: f32,
    pub forward_kick_steps: Vec<KickStep>,
    pub turn_kick_steps: Vec<KickStep>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct HeadMotionLimits {
    pub maximum_yaw: f32,
    pub maximum_pitch_at_center: f32,
    pub maximum_pitch_at_shoulder: f32,
    pub shoulder_yaw_position: f32,
    pub ear_shoulder_avoidance_width: f32,
    pub ear_shoulder_avoidance_pitch_penalty: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct DispatchingHeadInterpolator {
    pub maximum_yaw_velocity: f32,
    pub maximum_pitch_velocity: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct LookAt {
    pub minimum_bottom_focus_pitch: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct HeadMotion {
    pub outer_maximum_pitch: f32,
    pub inner_maximum_pitch: f32,
    pub outer_yaw: f32,
    pub maximum_velocity: HeadJoints,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct LookAround {
    pub time_at_each_position: Duration,
    pub middle_positions: HeadJoints,
    pub left_positions: HeadJoints,
    pub right_positions: HeadJoints,
    pub halfway_left_positions: HeadJoints,
    pub halfway_right_positions: HeadJoints,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct WhistleFilter {
    pub buffer_length: usize,
    pub minimum_detections: usize,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct SplNetwork {
    pub game_controller_return_message_interval: Duration,
    pub spl_striker_message_receive_timeout: Duration,
    pub spl_striker_message_send_interval: Duration,
    pub striker_trusts_team_ball: Duration,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct Vision {
    pub ball_detection: BallDetection,
    pub image_segmenter: ImageSegmenter,
    pub image_receiver: ImageReceiver,
    pub line_detection: LineDetection,
    pub field_border_detection: FieldBorderDetection,
    pub perspective_grid_candidates_provider: PerspectiveGridCandidatesProvider,
    pub camera_matrix_parameters: CameraMatrixParameters,
    pub projected_limbs: ProjectedLimbs,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct ImageSegmenter {
    pub horizontal_edge_threshold: i16,
    pub vertical_edge_threshold: i16,
    pub use_vertical_median: bool,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct FieldBorderDetection {
    pub min_points_per_line: usize,
    pub angle_threshold: f32,
    pub first_line_association_distance: f32,
    pub second_line_association_distance: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct BallDetection {
    pub minimal_radius: f32,
    pub preclassifier_neural_network: PathBuf,
    pub classifier_neural_network: PathBuf,
    pub positioner_neural_network: PathBuf,
    pub maximum_number_of_candidate_evaluations: usize,
    pub preclassifier_confidence_threshold: f32,
    pub classifier_confidence_threshold: f32,
    pub confidence_merge_factor: f32,
    pub correction_proximity_merge_factor: f32,
    pub image_containment_merge_factor: f32,
    pub cluster_merge_radius_factor: f32,
    pub ball_radius_enlargement_factor: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct ImageReceiver {
    pub resolution: i32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct LineDetection {
    pub allowed_line_length_in_field: Range<f32>,
    pub check_line_distance: bool,
    pub check_line_length: bool,
    pub check_line_segments_projection: bool,
    pub gradient_alignment: f32,
    pub maximum_distance_to_robot: f32,
    pub maximum_fit_distance_in_pixels: f32,
    pub maximum_gap_on_line: f32,
    pub maximum_number_of_lines: usize,
    pub maximum_projected_segment_length: f32,
    pub minimum_number_of_points_on_line: usize,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct OrientationFilter {
    pub acceleration_threshold: f32,
    pub delta_angular_velocity_threshold: f32,
    pub angular_velocity_bias_weight: f32,
    pub acceleration_weight: f32,
    pub falling_threshold: f32,
    pub force_sensitive_resistor_threshold: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct BallFilter {
    pub hypothesis_timeout: Duration,
    pub measurement_matching_distance: f32,
    pub hypothesis_merge_distance: f32,
    pub process_noise: Vector4<f32>,
    pub measurement_noise: Vector2<f32>,
    pub initial_covariance: Vector4<f32>,
    pub validity_exponential_decay_factor: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct StandUp {
    pub gyro_low_pass_filter_coefficient: f32,
    pub gyro_low_pass_filter_tolerance: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct FallStateEstimation {
    pub linear_acceleration_upright_threshold: Vector3<f32>,
    pub low_pass_filter_coefficient: f32,
    pub minimum_angle: Vector2<f32>,
    pub maximum_angle: Vector2<f32>,
    pub minimum_angular_velocity: Vector2<f32>,
    pub maximum_angular_velocity: Vector2<f32>,
    pub fallen_timeout: Duration,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct PerspectiveGridCandidatesProvider {
    pub minimum_radius: f32,
    pub fallback_radius: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct CameraMatrixParameters {
    pub extrinsic_rotations: Vector3<f32>,
    pub focal_lengths: Vector2<f32>,
    pub cc_optical_center: Point2<f32>,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct FallProtection {
    pub ground_impact_head_stiffness: f32,
    pub arm_stiffness: f32,
    pub left_arm_positions: ArmJoints,
    pub right_arm_positions: ArmJoints,
    pub leg_stiffness: f32,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize, SerializeHierarchy)]
pub struct ProjectedLimbs {
    pub torso_bounding_polygon: Vec<Point3<f32>>,
    pub lower_arm_bounding_polygon: Vec<Point3<f32>>,
    pub upper_arm_bounding_polygon: Vec<Point3<f32>>,
    pub knee_bounding_polygon: Vec<Point3<f32>>,
    pub foot_bounding_polygon: Vec<Point3<f32>>,
}
