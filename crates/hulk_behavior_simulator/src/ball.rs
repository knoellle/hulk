use bevy::prelude::*;
use coordinate_systems::Field;
use types::ball_position::BallPosition;

#[derive(Resource, Default)]
pub struct BallResource {
    pub state: Option<BallPosition<Field>>,
}

pub fn move_ball(mut ball: ResMut<BallResource>, time: Res<Time>) {
    if let Some(ball) = ball.state.as_mut() {
        ball.position += ball.velocity * time.delta_seconds();
        ball.velocity *= 0.98;
    }
}
