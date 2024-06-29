use crate::{ball::BallResource, cyclers::control::Database, robot::Robot, server};
use bevy::ecs::system::{Query, Res, ResMut, Resource};
use color_eyre::Result;
use coordinate_systems::Field;
use tokio_util::sync::CancellationToken;
use types::{ball_position::BallPosition, players::Players};

pub struct Frame {
    pub ball: Option<BallPosition<Field>>,
    pub robots: Players<Option<Database>>,
}

#[derive(Resource, Default)]
pub struct Recording {
    pub frames: Vec<Frame>,
}

pub fn frame_recorder(
    robots: Query<&Robot>,
    ball: Res<BallResource>,
    mut recording: ResMut<Recording>,
) {
    let mut players = Players::<Option<Database>>::default();
    for robot in &robots {
        players[robot.parameters.player_number] = Some(robot.database.clone())
    }
    recording.frames.push(Frame {
        robots: players,
        ball: ball.state,
    });
}

impl Recording {
    pub fn serve(&mut self) -> Result<()> {
        server::run(
            std::mem::take(&mut self.frames),
            Some("[::]:1337"),
            CancellationToken::new(),
        )
    }
}
