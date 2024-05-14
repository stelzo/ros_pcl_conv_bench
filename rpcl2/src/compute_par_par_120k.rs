mod processing;

use processing::*;

fn main() {
    let cloud_points = generate_random_pointcloud(120_000, -10_000.0, 10_000.0);
    for _ in 0..10 {
        if !roundtrip_computing_par_par(cloud_points.clone()) {
            println!("Error: roundtrip failed");
            break;
        }
    }
}
