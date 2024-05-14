mod processing;

use processing::*;

fn main() {
    let cloud_points = generate_random_pointcloud(1_500_000, f32::MIN / 2.0, f32::MAX / 2.0);
    for _ in 0..10 {
        if !roundtrip(cloud_points.clone()) {
            println!("Error: roundtrip failed");
            break;
        }
    }
}
