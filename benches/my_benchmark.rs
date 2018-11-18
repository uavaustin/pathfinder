#[macro_use]
extern crate criterion;
extern crate pathfinder;
use pathfinder::*;

use criterion::Criterion;

fn criterion_benchmark(c: &mut Criterion) {


	let j = Point::from_radians(8f64, -2f64, 0f32);
	let k = Point::from_radians(16f64, 6f64, 0f32);
	let ob = Obstacle::from_radians(15f64, 0f64, 5f32, 20f32);
	c.bench_function("perpendicular method", move |b| b.iter(|| Pathfinder::circle_intersect(&j, &k, &ob)));

}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);