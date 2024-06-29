use std::time::Duration;

use bevy::{
    app::{App, AppExit},
    ecs::event::ManualEventReader,
};
use hulk_behavior_simulator::{
    ball::{move_ball, BallResource},
    game_controller::GameController,
    recorder::{frame_recorder, Recording},
    robot::Robot,
    state::{cycle_robots, move_robots, Messages},
};
use spl_network_messages::{GameState, PlayerNumber};

#[derive(Default, Resource)]
struct GoldenGoal {
    cycle: u32,
    goals: u32,
}

fn init(mut commands: Commands) {
    commands.spawn(Robot::try_new(PlayerNumber::One).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Two).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Three).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Four).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Five).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Six).unwrap());
    commands.spawn(Robot::try_new(PlayerNumber::Seven).unwrap());
}

fn scenario(
    mut game_controller: ResMut<GameController>,
    mut robots: Query<&mut Robot>,
    mut ball: ResMut<BallResource>,
    mut scenario: ResMut<GoldenGoal>,
    mut exit: EventWriter<AppExit>,
) {
    if scenario.cycle == 100 {
        for mut robot in &mut robots {
            robot.is_penalized = false;
            ball.state.as_ref().inspect(|ball| {
                println!("{:?}", ball.position);
            });
        }
        game_controller.state.game_state = GameState::Ready;
    }
    if scenario.cycle == 1600 {
        game_controller.state.game_state = GameState::Set;
    }
    if scenario.cycle == 1700 {
        ball.state = Some(BallPosition::default());
    }

    if scenario.cycle % 2000 == 0 {
        game_controller.state.game_state = GameState::Playing;
    }

    if ball
        .state
        .as_ref()
        .is_some_and(|ball| ball.position.x().abs() > 4.5 && ball.position.y() < 0.75)
    {
        scenario.goals += 1;
        ball.state = None;
        game_controller.state.game_state = GameState::Set;
    }
    if scenario.goals > 0 || scenario.cycle >= 10_000 {
        exit.send(AppExit);
        println!("Done");
        // state.finished = true;
    }

    if scenario.cycle % 29 == 0 {
        println!("{}", scenario.cycle);
    }
    scenario.cycle += 1;
    // state.cycle(Duration::from_millis(12)).unwrap();
}

use bevy::prelude::*;
use types::ball_position::BallPosition;

fn update_time(mut time: ResMut<Time>) {
    time.advance_by(Duration::from_millis(12))
}

fn main() -> color_eyre::Result<()> {
    let mut app = App::new();
    app.add_plugins((
        TaskPoolPlugin::default(),
        TypeRegistrationPlugin,
        FrameCountPlugin,
    ))
    .insert_resource(GoldenGoal::default())
    .insert_resource(GameController::default())
    .insert_resource(BallResource::default())
    .insert_resource(Messages::default())
    .insert_resource(Recording::default())
    .insert_resource(Time::<()>::default())
    .add_systems(Startup, init)
    .add_systems(PreUpdate, update_time)
    .add_systems(Update, move_robots)
    .add_systems(Update, move_ball)
    .add_systems(Update, cycle_robots)
    .add_systems(Update, scenario)
    .add_systems(Update, frame_recorder);
    // todo!("replace with custom runner/simply call app.update manually");
    // app.run();
    let mut app_exit_event_reader = ManualEventReader::<AppExit>::default();
    loop {
        app.update();
        if let Some(app_exit_events) = app.world.get_resource_mut::<Events<AppExit>>() {
            if let Some(exit) = app_exit_event_reader.read(&app_exit_events).last() {
                println!("{:#?}", exit.clone());
                break;
            }
        }
    }

    dbg!(app.world)
        .get_resource_mut::<Recording>()
        .unwrap()
        .serve()
}
