#![recursion_limit = "256"]
#![feature(btree_cursors)]
use core::f32;
use std::time::SystemTime;

use color_eyre::eyre::Result;

use hardware::{
    ActuatorInterface, CameraInterface, IdInterface, NetworkInterface, PathsInterface,
    RecordingInterface, SensorInterface, SpeakerInterface, TimeInterface,
};
use types::{
    audio::SpeakerRequest,
    camera_position::CameraPosition,
    hardware::{Ids, Paths},
    joints::Joints,
    led::Leds,
    messages::{IncomingMessage, OutgoingMessage},
    sensor_data::SensorData,
    ycbcr422_image::YCbCr422Image,
};

mod delayer;

pub trait HardwareInterface:
    ActuatorInterface
    + IdInterface
    + NetworkInterface
    + PathsInterface
    + RecordingInterface
    + SensorInterface
    + SpeakerInterface
    + TimeInterface
{
}

include!(concat!(env!("OUT_DIR"), "/generated_code.rs"));

struct ImageExtractorHardwareInterface {
    ids: Ids,
}

impl CameraInterface for ImageExtractorHardwareInterface {
    fn read_from_camera(&self, _camera_position: CameraPosition) -> Result<YCbCr422Image> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl PathsInterface for ImageExtractorHardwareInterface {
    fn get_paths(&self) -> Paths {
        Paths {
            motions: "etc/motions".into(),
            neural_networks: "etc/neural_networks".into(),
            sounds: "etc/sounds".into(),
        }
    }
}

impl TimeInterface for ImageExtractorHardwareInterface {
    fn get_now(&self) -> SystemTime {
        SystemTime::now()
    }
}

impl HardwareInterface for ImageExtractorHardwareInterface {}

impl ActuatorInterface for ImageExtractorHardwareInterface {
    fn write_to_actuators(
        &self,
        _positions: Joints<f32>,
        _stiffnesses: Joints<f32>,
        _leds: Leds,
    ) -> Result<()> {
        Ok(())
    }
}

impl IdInterface for ImageExtractorHardwareInterface {
    fn get_ids(&self) -> Ids {
        self.ids.clone()
    }
}

impl NetworkInterface for ImageExtractorHardwareInterface {
    fn read_from_network(&self) -> Result<IncomingMessage> {
        panic!("Replayer cannot produce data from hardware")
    }

    fn write_to_network(&self, _message: OutgoingMessage) -> Result<()> {
        Ok(())
    }
}

impl RecordingInterface for ImageExtractorHardwareInterface {
    fn should_record(&self) -> bool {
        false
    }

    fn set_whether_to_record(&self, _enable: bool) {}
}

impl SensorInterface for ImageExtractorHardwareInterface {
    fn read_from_sensors(&self) -> Result<SensorData> {
        panic!("Replayer cannot produce data from hardware")
    }
}

impl SpeakerInterface for ImageExtractorHardwareInterface {
    fn write_to_speakers(&self, _request: SpeakerRequest) {}
}

fn main() -> Result<()> {
    delayer::delayer()
}
