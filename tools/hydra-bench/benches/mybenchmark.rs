use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};

use hydra_bench::{run_inference, sample_image, setup};

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("fib 20", |b| {
        b.iter_batched_ref(
            || {
                let session =
                    setup("./yolo26m-tuned_pose-tuned-hydra-nv12.onnx", "./cache/").unwrap();
                let image = sample_image();
                (session, image)
            },
            |(session, image)| drop(run_inference(session, black_box(image))),
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
