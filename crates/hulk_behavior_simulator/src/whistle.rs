use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct WhistleResource {
    pub is_whistling: bool,
    pub has_whistled: bool,
    pub timer: Option<Timer>,
}

pub fn whistle(mut whistle: ResMut<WhistleResource>) {
    whistle.is_whistling = true;
    whistle.timer = Some(Timer::from_seconds(1.0, TimerMode::Once));
}
pub fn breathe(mut whistle: ResMut<WhistleResource>) {
    whistle.has_whistled = true;
    whistle.is_whistling = false;
}
