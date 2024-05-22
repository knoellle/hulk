use std::time::{Duration, SystemTime};

use crate::{cyclers::control::Database, server, state::Ball};
use color_eyre::Result;
use tokio_util::sync::CancellationToken;
use types::players::Players;

use crate::state::State;

pub struct Frame {
    pub ball: Option<Ball>,
    pub robots: Players<Option<Database>>,
}

pub trait Scenario {
    fn init(&mut self, state: &mut State) -> Result<()>;

    fn cycle(&mut self, state: &mut State) -> Result<()> {
        state.cycle(Duration::from_millis(12))
    }

    fn run(&mut self) -> Result<()> {
        let mut state = State::default();
        self.init(&mut state)?;
        let start = SystemTime::now();
        while self.should_continue(&mut state) {
            self.cycle(&mut state)?;
        }
        let duration = start.elapsed().expect("time ran backwards");
        println!("Took {:.2}s", duration.as_secs_f32());
        let cycles = state.cycle_count;
        println!(
            "{} cycles, {:.2} cycles/s",
            cycles,
            cycles as f32 / duration.as_secs_f32()
        );

        Ok(())
    }

    fn should_continue(&mut self, state: &mut State) -> bool {
        !state.finished && state.cycle_count < 10_000
    }
}

#[derive(Default)]
pub struct Recorder {
    pub frames: Vec<Frame>,
}

impl Plugin for Recorder {
    fn cycle(&mut self, state: &mut State) -> Result<()> {
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

    fn finish(&mut self, _state: &mut State) -> Result<()> {
        server::run(
            std::mem::take(&mut self.frames),
            Some("[::]:1337"),
            CancellationToken::new(),
        )
    }
}

pub trait Plugin {
    fn init(&mut self, _state: &mut State) -> Result<()> {
        Ok(())
    }

    fn cycle(&mut self, state: &mut State) -> Result<()>;

    fn finish(&mut self, _state: &mut State) -> Result<()> {
        Ok(())
    }
}

#[derive(Default)]
pub struct Simulator {
    state: State,
    plugins: Vec<Box<dyn Plugin>>,
}

impl Simulator {
    pub fn register(&mut self, mut plugin: impl Plugin + 'static) -> Result<()> {
        plugin.init(&mut self.state)?;
        self.plugins.push(Box::new(plugin));
        Ok(())
    }

    pub fn cycle(&mut self) -> Result<()> {
        for plugin in &mut self.plugins {
            plugin.cycle(&mut self.state)?;
        }

        Ok(())
    }

    pub fn run(&mut self) -> Result<()> {
        let start = SystemTime::now();

        while !self.state.finished {
            self.cycle()?;
        }

        let duration = start.elapsed().expect("time ran backwards");
        println!("Took {:.2}s", duration.as_secs_f32());
        let cycles = self.state.cycle_count;
        println!(
            "{} cycles, {:.2} cycles/s",
            cycles,
            cycles as f32 / duration.as_secs_f32()
        );

        for plugin in &mut self.plugins {
            plugin.finish(&mut self.state)?;
        }

        Ok(())
    }
}
