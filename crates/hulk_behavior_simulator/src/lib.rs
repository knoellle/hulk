use hardware::{NetworkInterface, RecordingInterface, TimeInterface};
use interfake::FakeDataInterface;

pub mod ball;
pub mod fake_data;
pub mod game_controller;
pub mod interfake;
pub mod recorder;
pub mod robot;
pub mod server;
pub mod state;

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));

pub trait HardwareInterface:
    TimeInterface + NetworkInterface + RecordingInterface + FakeDataInterface
{
}
