use bevy::{
    app::{App, AppExit, First, Plugin, Update},
    core::{FrameCountPlugin, TaskPoolPlugin, TypeRegistrationPlugin},
    ecs::{
        event::{Events, ManualEventReader},
        schedule::IntoSystemConfigs,
    },
    time::Time,
};
use color_eyre::{eyre::eyre, Result};

use crate::{
    autoref::{autoref, autoref_plugin},
    ball::{move_ball, BallResource},
    game_controller::{game_controller_plugin, GameController},
    recorder::Recording,
    robot::{cycle_robots, move_robots, Messages},
    time::{update_time, Ticks},
    whistle::WhistleResource,
};

#[derive(Default, Copy, Clone)]
pub struct SimulatorPlugin {
    pub use_recording: bool,
}

impl SimulatorPlugin {
    pub fn with_recording(mut self, use_recording: bool) -> Self {
        self.use_recording = use_recording;

        self
    }
}

impl Plugin for SimulatorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            TaskPoolPlugin::default(),
            TypeRegistrationPlugin,
            FrameCountPlugin,
        ))
        .add_plugins(autoref_plugin)
        .add_plugins(game_controller_plugin)
        .insert_resource(GameController::default())
        .insert_resource(BallResource::default())
        .insert_resource(WhistleResource::default())
        .insert_resource(Messages::default())
        .insert_resource(Time::<()>::default())
        .insert_resource(Time::<Ticks>::default())
        .add_systems(First, update_time)
        .add_systems(Update, cycle_robots.before(move_robots).after(autoref))
        .add_systems(Update, move_robots)
        .add_systems(Update, move_ball.after(move_robots));

        if self.use_recording {
            app.add_plugins(crate::recorder::recording_plugin);
        }
    }
}

pub trait AppExt {
    fn run_to_completion(&mut self) -> Result<()>;
}

impl AppExt for App {
    fn run_to_completion(&mut self) -> Result<()> {
        let mut app_exit_event_reader = ManualEventReader::<AppExit>::default();
        let exit = loop {
            self.update();
            if let Some(exit) = self
                .world_mut()
                .get_resource_mut::<Events<AppExit>>()
                .and_then(|events| app_exit_event_reader.read(&events).last().cloned())
            {
                break exit;
            }
        };
        if let Some(mut recording) = self.world_mut().get_resource_mut::<Recording>() {
            println!("serving {} frames", recording.frames.len());
            recording.serve()?
        }

        match exit {
            AppExit::Success => Ok(()),
            AppExit::Error(code) => Err(eyre!("Scenario exited with error code {code}")),
        }
    }
}
