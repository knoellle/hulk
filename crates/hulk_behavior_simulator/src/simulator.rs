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
    pub frames: Vec<Frame>,
    pub callback: Option<Callback>,
}

impl Simulator {
    pub fn cycle(&mut self, state: &mut State) -> Result<()> {
        state.cycle(Duration::from_millis(12))?;

        let mut robots = Players::<Option<Database>>::default();
        for (player_number, robot) in &state.robots {
            robots[*player_number] = Some(robot.database.clone())
        }
        self.frames.push(Frame {
            robots,
            ball: state.ball.clone(),
        });

        Ok(())
    }
}
