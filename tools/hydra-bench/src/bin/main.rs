use clap::Parser;
use color_eyre::{Result, eyre::Context};
use hydra_bench::{CliArguments, run_inference, sample_image, setup};

fn main() -> Result<()> {
    let args = CliArguments::parse();
    color_eyre::install()?;
    std::fs::create_dir_all(&args.cache_path).wrap_err("failed to create cache path")?;

    let mut session = setup(args)?;

    let sample_image = sample_image();
    let _ = run_inference(&mut session, &sample_image)?;

    eprintln!("object detection setup complete");

    Ok(())
}
