use anyhow::Result;
use module_derive::module;
use nalgebra::{point, Point2};
use spl_network::{GamePhase, Team};
use types::{
    CameraMatrices, FieldDimensions, FilteredGameState, KickDecision, MotionCommand, PathObstacle,
    ProjectedLimbs, Role, WorldState,
};

use crate::framework::configuration;

use super::{
    action::Action,
    defend::Defend,
    dribble, fall_safely,
    head::LookAction,
    jump, lost_ball, penalize, prepare_jump, search, sit_down, stand, stand_up, support_striker,
    unstiff, walk_to_kick_off,
    walk_to_pose::{WalkAndStand, WalkPathPlanner},
};

pub struct Behavior {
    last_motion_command: MotionCommand,
    absolute_last_known_ball_position: Point2<f32>,
}

#[module(control)]
#[input(path = world_state, data_type = WorldState, required)]
#[input(path = camera_matrices, data_type = CameraMatrices)]
#[input(path = projected_limbs, data_type = ProjectedLimbs)]
#[parameter(path = control.behavior, data_type = configuration::Behavior)]
#[parameter(path = field_dimensions, data_type = FieldDimensions)]
#[parameter(path = control.behavior.lost_ball, data_type=configuration::LostBall, name=lost_ball_parameters)]
#[additional_output(path = path_obstacles, data_type = Vec<PathObstacle>)]
#[additional_output(path = kick_decisions, data_type = Vec<KickDecision>)]
#[additional_output(path = kick_targets, data_type = Vec<Point2<f32>>)]
#[main_output(data_type = MotionCommand)]
impl Behavior {}

impl Behavior {
    fn new(_context: NewContext) -> anyhow::Result<Self> {
        Ok(Self {
            last_motion_command: MotionCommand::Unstiff,
            absolute_last_known_ball_position: point![0.0, 0.0],
        })
    }

    fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let world_state = context.world_state;

        if let Some(command) = &context.behavior.injected_motion_command {
            return Ok(MainOutputs {
                motion_command: Some(command.clone()),
            });
        }

        if let (Some(ball_state), Some(robot_to_field)) =
            (&world_state.ball, world_state.robot.robot_to_field)
        {
            self.absolute_last_known_ball_position = robot_to_field * ball_state.position;
        }

        let mut actions = vec![
            Action::Unstiff,
            Action::SitDown,
            Action::Penalize,
            Action::FallSafely,
            Action::StandUp,
            Action::Stand,
        ];

        match world_state.robot.role {
            Role::DefenderLeft => actions.push(Action::DefendLeft),
            Role::DefenderRight => actions.push(Action::DefendRight),
            Role::Keeper => match world_state.game_phase {
                GamePhase::PenaltyShootout { .. } => {
                    actions.push(Action::Jump);
                    actions.push(Action::PrepareJump);
                }
                _ => actions.push(Action::DefendGoal),
            },
            Role::Loser => actions.push(Action::SearchForLostBall),
            Role::ReplacementKeeper => actions.push(Action::DefendGoal),
            Role::Searcher => actions.push(Action::Search),
            Role::Striker => match world_state.filtered_game_state {
                None | Some(FilteredGameState::Playing { ball_is_free: true }) => {
                    actions.push(Action::Dribble);
                }
                Some(FilteredGameState::Ready {
                    kicking_team: Team::Hulks,
                }) => {
                    actions.push(Action::WalkToKickOff);
                }
                _ => {
                    actions.push(Action::DefendKickOff);
                }
            },
            Role::StrikerSupporter => {
                actions.push(Action::SupportStriker);
            }
        };

        let walk_path_planner = WalkPathPlanner::new(
            context.field_dimensions,
            &world_state.obstacles,
            &context.behavior.path_planning,
        );
        let walk_and_stand = WalkAndStand::new(
            world_state,
            &context.behavior.walk_and_stand,
            &walk_path_planner,
            &self.last_motion_command,
        );
        let look_action = LookAction::new(
            world_state,
            context.field_dimensions,
            &context.behavior.look_action,
        );
        let defend = Defend::new(
            world_state,
            context.field_dimensions,
            &context.behavior.role_positions,
            &walk_and_stand,
            &look_action,
        );

        let motion_command = actions
            .iter()
            .find_map(|action| match action {
                Action::Unstiff => unstiff::execute(world_state),
                Action::SitDown => sit_down::execute(world_state),
                Action::Penalize => penalize::execute(world_state),
                Action::FallSafely => fall_safely::execute(world_state),
                Action::StandUp => stand_up::execute(world_state),
                Action::DefendGoal => defend.goal(&mut context.path_obstacles),
                Action::DefendKickOff => defend.kick_off(&mut context.path_obstacles),
                Action::DefendLeft => defend.left(&mut context.path_obstacles),
                Action::DefendRight => defend.right(&mut context.path_obstacles),
                Action::Stand => stand::execute(world_state),
                Action::Dribble => dribble::execute(
                    world_state,
                    context.field_dimensions,
                    &context.behavior.dribbling,
                    &walk_path_planner,
                    &mut context.path_obstacles,
                    &mut context.kick_targets,
                    &mut context.kick_decisions,
                ),
                Action::Jump => jump::execute(world_state),
                Action::PrepareJump => prepare_jump::execute(world_state),
                Action::Search => search::execute(
                    world_state,
                    &walk_path_planner,
                    &walk_and_stand,
                    context.field_dimensions,
                    &context.behavior.search,
                    &mut context.path_obstacles,
                ),
                Action::SearchForLostBall => lost_ball::execute(
                    world_state,
                    self.absolute_last_known_ball_position,
                    &walk_path_planner,
                    context.lost_ball_parameters,
                    &mut context.path_obstacles,
                ),
                Action::SupportStriker => support_striker::execute(
                    world_state,
                    context.field_dimensions,
                    &context.behavior.role_positions,
                    &walk_and_stand,
                    &look_action,
                    &mut context.path_obstacles,
                ),
                Action::WalkToKickOff => walk_to_kick_off::execute(
                    world_state,
                    &walk_and_stand,
                    &look_action,
                    &mut context.path_obstacles,
                ),
            })
            .unwrap_or_else(|| {
                panic!(
                    "There has to be at least one action available, world_state: {:#?}",
                    world_state
                )
            });

        self.last_motion_command = motion_command.clone();

        Ok(MainOutputs {
            motion_command: Some(motion_command),
        })
    }
}
