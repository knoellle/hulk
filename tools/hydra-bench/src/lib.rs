use std::path::PathBuf;

use clap::Parser;
use color_eyre::{Result, eyre::Context};
use ndarray::Array3;
use ort::{
    execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider},
    inputs,
    session::{Session, SessionOutputs, builder::GraphOptimizationLevel},
    value::TensorRef,
};

#[derive(Debug, Parser)]
pub struct CliArguments {
    /// Path to onnx model
    pub onnx_path: PathBuf,

    /// Path to cache folder
    #[arg(long, default_value = "/home/booster/hulk/etc/neural_networks/")]
    pub cache_path: PathBuf,
}

pub fn run_inference<'a>(
    session: &'a mut Session,
    sample_image: &Array3<u8>,
) -> Result<SessionOutputs<'a>> {
    Ok(session
        .run(inputs!["raw_bytes_input" => TensorRef::from_array_view(sample_image.view())?])?)
}

pub fn setup(args: CliArguments) -> Result<Session, color_eyre::eyre::Error> {
    let tensor_rt = TensorRTExecutionProvider::default()
        .with_device_id(0)
        .with_fp16(true)
        .with_engine_cache(true)
        .with_engine_cache_path(args.cache_path.display())
        .build()
        .error_on_failure();
    let cuda = CUDAExecutionProvider::default().build();
    let session = Session::builder()?
        .with_execution_providers([tensor_rt, cuda])?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(2)?
        .commit_from_file(args.onnx_path)?;
    Ok(session)
}

pub fn sample_image() -> Array3<u8> {
    const IMAGE_WIDTH: usize = 544;
    const IMAGE_HEIGHT: usize = 448;
    Array3::<u8>::default([IMAGE_HEIGHT / 2, IMAGE_WIDTH / 2, 6])
}
