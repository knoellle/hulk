use behavior_simulator::{server, simulator::Simulator, state::Ball};
use spl_network_messages::{GameState, PlayerNumber, Team};
use tokio::time::Instant;
use tokio_util::sync::CancellationToken;
use types::filtered_game_state::FilteredGameState;

fn main() -> color_eyre::Result<()> {
    let mut simulator = Simulator::default();

    for player_number in [
        PlayerNumber::One,
        PlayerNumber::Two,
        PlayerNumber::Three,
        PlayerNumber::Four,
        PlayerNumber::Five,
        PlayerNumber::Six,
        PlayerNumber::Seven,
    ] {
        simulator.state.spawn_robot(player_number)?.is_penalized = false;
    }

    simulator.state.game_controller_state.game_state = GameState::Ready;
    simulator.state.filtered_game_state = FilteredGameState::Ready {
        kicking_team: Team::Hulks,
    };

    let start = Instant::now();
    for cycle in 0..10000 {
        if cycle % 1000 == 0 && simulator.state.ball.is_none() {
            simulator.state.ball = Some(Ball::default());
        }

        if cycle == 1600 {
            simulator.state.filtered_game_state = FilteredGameState::Set;
        }
        if cycle % 2000 == 0 {
            simulator.state.filtered_game_state = FilteredGameState::Playing { ball_is_free: true };
        }

        if simulator
            .state
            .ball
            .as_ref()
            .is_some_and(|ball| ball.position.x.abs() > 4.5 && ball.position.y < 0.75)
        {
            simulator.state.ball = None;
            simulator.state.filtered_game_state = FilteredGameState::Set;
        }

        simulator.cycle()?;
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
