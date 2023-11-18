use std::{sync::Arc, time::Duration};

use crate::{cyclers::control::Database, state::Ball};
use color_eyre::Result;
use types::players::Players;

use crate::state::State;

pub struct Frame {
    pub ball: Option<Ball>,
    pub robots: Players<Option<Database>>,
}

pub type Callback = Arc<dyn FnMut(&mut State) + Send + Sync>;

#[derive(Default)]
pub struct Simulator {
    pub state: State,
    pub frames: Vec<Frame>,
    pub callback: Option<Callback>,
}

impl Simulator {
    pub fn run(&mut self) -> Result<()> {
        loop {
            self.cycle()?;
            if self.state.finished {
                break;
            }
        }

        Ok(())
    }

    pub fn register_callback(&mut self, f: Callback) {
        self.callback.replace(f);
    }

    pub fn cycle(&mut self) -> Result<()> {
        self.state.cycle(Duration::from_millis(12))?;

        let mut robots = Players::<Option<Database>>::default();
        for (player_number, robot) in &self.state.robots {
            robots[*player_number] = Some(robot.database.clone())
        }
        self.frames.push(Frame {
            robots,
            ball: self.state.ball.clone(),
        });

        if self.state.cycle_count > 10_000 {
            self.state.finished = true;
        }

        Ok(())
    }
}
