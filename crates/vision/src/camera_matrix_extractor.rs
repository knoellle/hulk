use std::fs::read_to_string;

use color_eyre::{eyre::Context, Result};
use context_attribute::context;
use framework::MainOutput;
use serde::{Deserialize, Serialize};
use types::{
    camera_matrix::{CameraMatrices, CameraMatrix},
    camera_position::CameraPosition,
};

#[derive(Deserialize, Serialize)]
pub struct CameraMatrixExtractor {}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    camera_matrices: RequiredInput<Option<CameraMatrices>, "Control", "camera_matrices?">,
    camera_position: Parameter<CameraPosition, "image_receiver.$cycler_instance.camera_position">,
    fake_data_path: Parameter<String, "fake_data_path">,
    fake_data_index: Parameter<usize, "fake_data_index">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub camera_matrix: MainOutput<Option<CameraMatrix>>,
}

impl CameraMatrixExtractor {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let camera_matrix = match context.camera_position {
            CameraPosition::Top => &context.camera_matrices.top,
            CameraPosition::Bottom => &context.camera_matrices.bottom,
        }
        .clone();

        let path = format!(
            "{}{}.camera_matrix.json",
            context.fake_data_path, context.fake_data_index
        );
        let camera_matrix =
            serde_json::from_str(&read_to_string(&path).with_context(|| path.to_string())?)
                .unwrap();

        Ok(MainOutputs {
            camera_matrix: Some(camera_matrix).into(),
        })
    }
}
