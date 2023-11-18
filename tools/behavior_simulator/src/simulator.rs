use std::time::Duration;

use crate::{cycler::Database, server, state::Ball};
use color_eyre::Result;
use tokio_util::sync::CancellationToken;
use types::players::Players;

use crate::state::State;

pub struct Frame {
    pub ball: Option<Ball>,
    pub robots: Players<Option<Database>>,
}

pub trait Scenario {
    fn cycle(&mut self) -> Result<()> {
        self.state().cycle(Duration::from_millis(12))
    }

    fn state(&mut self) -> &mut State;

    fn run(&mut self) -> Result<()> {
        let mut recorder = Recorder::default();
        while self.should_continue() {
            self.cycle()?;
            recorder.record(self.state());
        }

        server::run(recorder.frames, Some("[::]:1337"), CancellationToken::new())?;

        Ok(())
    }

    fn should_continue(&mut self) -> bool {
        self.state().cycle_count < 10_000
    }
}

#[derive(Default)]
pub struct Recorder {
    pub frames: Vec<Frame>,
}

impl Recorder {
    pub fn record(&mut self, state: &mut State) {
        let mut robots = Players::<Option<Database>>::default();
        for (player_number, robot) in &state.robots {
            robots[*player_number] = Some(robot.database.clone())
        }
        self.frames.push(Frame {
            robots,
            ball: state.ball.clone(),
        });
    }
}
