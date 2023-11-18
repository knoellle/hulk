use std::time::Duration;

use behavior_simulator::{
    server,
    simulator::{Recorder, Scenario},
    state::{Ball, State},
};
use spl_network_messages::{PlayerNumber, Team};
use tokio::time::Instant;
use tokio_util::sync::CancellationToken;
use types::filtered_game_state::FilteredGameState;

#[derive(Default)]
struct GoldenGoal {
    state: State,
    goals: u32,
}

impl Scenario for GoldenGoal {
    fn cycle(&mut self) -> color_eyre::eyre::Result<()> {
        let cycle = self.state.cycle_count;
        let state = self.state;
        if cycle == 100 {
            for robot in state.robots.values_mut() {
                robot.is_penalized = false;
            }
            state.filtered_game_state = FilteredGameState::Ready {
                kicking_team: Team::Hulks,
            }
        }
        if cycle == 1600 {
            state.filtered_game_state = FilteredGameState::Set;
        }
        if cycle == 1700 {
            state.ball = Some(Ball::default());
        }

        if cycle % 2000 == 0 {
            state.filtered_game_state = FilteredGameState::Playing { ball_is_free: true };
        }

        if state
            .ball
            .as_ref()
            .is_some_and(|ball| ball.position.x.abs() > 4.5 && ball.position.y < 0.75)
        {
            self.goals += 1;
            state.ball = None;
            state.filtered_game_state = FilteredGameState::Set;
        }
        if self.goals > 0 {
            break;
        }

        state.cycle(Duration::from_millis(12))?;

        Ok(())
    }

    fn state(&mut self) -> &mut State {
        &mut self.state
    }
}

fn main() -> color_eyre::Result<()> {
    let mut scenario = GoldenGoal::default();
    scenario.run();
    return Ok(());

    let mut simulator = Recorder::default();
    let mut state = State::default();

    for player_number in [
        PlayerNumber::One,
        PlayerNumber::Two,
        PlayerNumber::Three,
        PlayerNumber::Four,
        PlayerNumber::Five,
        PlayerNumber::Six,
        PlayerNumber::Seven,
    ] {
        state.spawn_robot(player_number)?;
    }

    let mut goals = 0;

    let start = Instant::now();
    for cycle in 0..10000 {
        if cycle == 100 {
            for robot in state.robots.values_mut() {
                robot.is_penalized = false;
            }
            state.filtered_game_state = FilteredGameState::Ready {
                kicking_team: Team::Hulks,
            }
        }
        if cycle == 1600 {
            state.filtered_game_state = FilteredGameState::Set;
        }
        if cycle == 1700 {
            state.ball = Some(Ball::default());
        }

        if cycle % 2000 == 0 {
            state.filtered_game_state = FilteredGameState::Playing { ball_is_free: true };
        }

        if state
            .ball
            .as_ref()
            .is_some_and(|ball| ball.position.x.abs() > 4.5 && ball.position.y < 0.75)
        {
            goals += 1;
            state.ball = None;
            state.filtered_game_state = FilteredGameState::Set;
        }
        if goals > 0 {
            break;
        }

        state.cycle(Duration::from_millis(12))?;
        simulator.record(&mut state);
    }
    let duration = Instant::now() - start;
    println!("Took {:.2} seconds", duration.as_secs_f32());
    println!("Frames: {}", simulator.frames.len());

    server::run(
        simulator.frames,
        Some("[::]:1337"),
        CancellationToken::new(),
    )?;

    Ok(())
}
