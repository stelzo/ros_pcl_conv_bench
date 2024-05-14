#![allow(dead_code)]

use ros_pointcloud2::prelude::*;

use rand::Rng;

pub fn distance_to_origin(point: &PointXYZ) -> f32 {
    ((point.x.powi(2)) + (point.y.powi(2)) + (point.z.powi(2))).sqrt()
}

pub fn dot_product(point1: &PointXYZ, point2: &PointXYZ) -> f32 {
    point1.x * point2.x + point1.y * point2.y + point1.z * point2.z
}

pub fn cross_product(point1: &PointXYZ, point2: &PointXYZ) -> PointXYZ {
    PointXYZ {
        x: point1.y * point2.z - point1.z * point2.y,
        y: point1.z * point2.x - point1.x * point2.z,
        z: point1.x * point2.y - point1.y * point2.x,
    }
}

pub fn scalar_multiply(point: &PointXYZ, scalar: f32) -> PointXYZ {
    PointXYZ {
        x: point.x * scalar,
        y: point.y * scalar,
        z: point.z * scalar,
    }
}

pub fn magnitude_squared(point: &PointXYZ) -> f32 {
    (point.x.powi(2)) + (point.y.powi(2)) + (point.z.powi(2))
}

pub fn reflection_through_plane(
    point: &PointXYZ,
    normal: &PointXYZ,
    point_on_plane: &PointXYZ,
) -> PointXYZ {
    let v = PointXYZ {
        x: point.x
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
        y: point.y
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
        z: point.z
            - 2.0
                * ((point.x - point_on_plane.x) * normal.x
                    + (point.y - point_on_plane.y) * normal.y
                    + (point.z - point_on_plane.z) * normal.z),
    };
    v
}

pub fn rotation_about_x(point: &PointXYZ, angle: f32) -> PointXYZ {
    let c = f32::cos(angle);
    let s = f32::sin(angle);
    PointXYZ {
        x: point.x,
        y: point.y * c - point.z * s,
        z: point.y * s + point.z * c,
    }
}

pub fn closest_point_on_line(
    point: &PointXYZ,
    line_point: &PointXYZ,
    line_direction: &PointXYZ,
) -> PointXYZ {
    let v = PointXYZ {
        x: line_point.x
            + (line_point.x - point.x) * ((line_point.x - point.x).powi(2))
                / ((line_direction.x * 2.0).powi(2))
            + (line_direction.y * 2.0) * (point.z - line_point.z)
                / ((line_direction.z * 2.0).powi(2)),
        y: line_point.y
            + (line_point.y - point.y) * ((line_point.y - point.y).powi(2))
                / ((line_direction.y * 2.0).powi(2))
            + (line_direction.x * 2.0) * (point.x - line_point.x)
                / ((line_direction.x * 2.0).powi(2)),
        z: line_point.z
            + (line_point.z - point.z) * ((line_point.z - point.z).powi(2))
                / ((line_direction.z * 2.0).powi(2))
            + (line_direction.y * 2.0) * (point.y - line_point.y)
                / ((line_direction.y * 2.0).powi(2)),
    };
    v
}

fn minus(point1: &PointXYZ, point2: &PointXYZ) -> PointXYZ {
    PointXYZ {
        x: point1.x - point2.x,
        y: point1.y - point2.y,
        z: point1.z - point2.z,
    }
}

pub fn generate_random_pointcloud(num_points: usize, min: f32, max: f32) -> Vec<PointXYZ> {
    let mut rng = rand::thread_rng();
    let mut pointcloud = Vec::with_capacity(num_points);
    for _ in 0..num_points {
        let point = PointXYZ {
            x: rng.gen_range(min..max),
            y: rng.gen_range(min..max),
            z: rng.gen_range(min..max),
        };
        pointcloud.push(point);
    }
    pointcloud
}

pub fn heavy_computing(point: &PointXYZ, iterations: u32) -> f32 {
    let mut result = distance_to_origin(point);
    for _ in 0..iterations {
        result += dot_product(
            point,
            &PointXYZ {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        );
        result += cross_product(
            point,
            &PointXYZ {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
        )
        .x;
        result = result + (result * 10.0);
        if result > 0.0 {
            result = result.sqrt();
        }
        let reflected_point = reflection_through_plane(
            point,
            &PointXYZ {
                x: 7.0,
                y: 8.0,
                z: 9.0,
            },
            &PointXYZ {
                x: 3.0,
                y: 4.0,
                z: 5.0,
            },
        );
        let rotated_point = rotation_about_x(
            &PointXYZ {
                x: 10.0,
                y: 11.0,
                z: 12.0,
            },
            3.14159,
        );

        result += magnitude_squared(&minus(&reflected_point, &rotated_point));
    }

    result
}

pub fn roundtrip_vec(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total: Vec<PointXYZ> = internal_msg.try_into_vec().unwrap();
    orig_len == total.len()
}

pub fn roundtrip(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .collect::<Vec<PointXYZ>>();
    orig_len == total.len()
}

fn point_inside_x_range(point: &PointXYZ, min: f32, max: f32) -> bool {
    point.x >= min && point.x <= max
}

pub fn roundtrip_filter_vec(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total: Vec<PointXYZ> = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| point_inside_x_range(point, -5.0, 5.0))
        .collect();
    orig_len > total.len()
}

pub fn roundtrip_filter(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total: Vec<PointXYZ> = internal_msg
        .try_into_iter()
        .unwrap()
        .filter(|point: &PointXYZ| point_inside_x_range(point, -5.0, 5.0))
        .collect();
    orig_len > total.len()
}

pub fn roundtrip_filter_par_par(cloud: Vec<PointXYZ>) -> bool {
    let orig_len = cloud.len();
    let internal_msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter()).unwrap();
    let total: Vec<PointXYZ> = internal_msg
        .try_into_par_iter()
        .unwrap()
        .filter(|point: &PointXYZ| point_inside_x_range(point, -5.0, 5.0))
        .collect();
    orig_len > total.len()
}

pub fn roundtrip_computing(cloud: Vec<PointXYZ>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_iter(cloud).unwrap();
    let total = internal_msg
        .try_into_iter()
        .unwrap()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum::<f32>();
    total > 0.0
}

pub fn roundtrip_computing_vec(cloud: Vec<PointXYZ>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_vec(cloud).unwrap();
    let total: f32 = internal_msg
        .try_into_vec()
        .unwrap()
        .into_iter()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum();
    total > 0.0
}

pub fn roundtrip_computing_par_par(cloud: Vec<PointXYZ>) -> bool {
    let internal_msg = PointCloud2Msg::try_from_par_iter(cloud.into_par_iter()).unwrap();
    let total = internal_msg
        .try_into_par_iter()
        .unwrap()
        .map(|point: PointXYZ| heavy_computing(&point, 100))
        .sum::<f32>();
    total > 0.0
}
