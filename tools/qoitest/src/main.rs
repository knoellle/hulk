use std::path::Path;

use image::{ImageFormat, RgbImage};
use qoi::encode_to_vec;
use rayon::prelude::{ParallelBridge, ParallelIterator};
use types::color::YCbCr422;
use types::ycbcr422_image::YCbCr422Image;

fn as_bytes(v: &[YCbCr422]) -> &[u8] {
    unsafe {
        std::slice::from_raw_parts(
            v.as_ptr() as *const u8,
            v.len() * std::mem::size_of::<YCbCr422>(),
        )
    }
}

fn check_file(path: &Path) -> Option<usize> {
    println!("{path:?}");
    let ycbcr = YCbCr422Image::load_from_444_png(path).ok()?;
    let rgb: RgbImage = ycbcr.into();
    let bytes = rgb.as_raw();
    let encoded = encode_to_vec(bytes, rgb.width(), rgb.height()).ok()?;
    println!("Original: {}", bytes.len());
    println!("Encoded : {}", encoded.len());
    Some(encoded.len())
}

fn main() {
    let path = Path::new("../../tools/machine-learning/data/HULKs/events/2019-10-30_testGame/raw/10.1.24.37/replay_108159/");
    let sizes: Vec<usize> = path
        .read_dir()
        .unwrap()
        .par_bridge()
        .flat_map(|path| check_file(&path.unwrap().path()))
        .collect();
    println!("Count  : {}", sizes.len());
    println!("Average: {}", sizes.iter().sum::<usize>() / sizes.len());
}
