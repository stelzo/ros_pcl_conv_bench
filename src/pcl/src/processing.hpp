#pragma once

#include <cmath>
#include <vector>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <functional>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <omp.h>

float distance_to_origin(const pcl::PointXYZ &point)
{
    return std::sqrt((point.x * point.x) + (point.y * point.y) + (point.z * point.z));
}

float dot_product(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
{
    return point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
}

pcl::PointXYZ cross_product(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
{
    return pcl::PointXYZ{
        point1.y * point2.z - point1.z * point2.y,
        point1.z * point2.x - point1.x * point2.z,
        point1.x * point2.y - point1.y * point2.x};
}

pcl::PointXYZ scalar_multiply(const pcl::PointXYZ &point, float scalar)
{
    return pcl::PointXYZ{
        point.x * scalar,
        point.y * scalar,
        point.z * scalar};
}

float magnitude_squared(const pcl::PointXYZ &point)
{
    return (point.x * point.x) + (point.y * point.y) + (point.z * point.z);
}

pcl::PointXYZ reflection_through_plane(const pcl::PointXYZ &point, const pcl::PointXYZ &normal, const pcl::PointXYZ &point_on_plane)
{
    pcl::PointXYZ v{
        point.x - 2.0 * ((point.x - point_on_plane.x) * normal.x + (point.y - point_on_plane.y) * normal.y + (point.z - point_on_plane.z) * normal.z),
        point.y - 2.0 * ((point.x - point_on_plane.x) * normal.x + (point.y - point_on_plane.y) * normal.y + (point.z - point_on_plane.z) * normal.z),
        point.z - 2.0 * ((point.x - point_on_plane.x) * normal.x + (point.y - point_on_plane.y) * normal.y + (point.z - point_on_plane.z) * normal.z)};
    return v;
}

pcl::PointXYZ rotation_about_x(const pcl::PointXYZ &point, float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    return pcl::PointXYZ{
        point.x,
        point.y * c - point.z * s,
        point.y * s + point.z * c};
}

pcl::PointXYZ closest_point_on_line(const pcl::PointXYZ &point, const pcl::PointXYZ &line_point, const pcl::PointXYZ &line_direction)
{
    pcl::PointXYZ v{
        line_point.x + (line_point.x - point.x) * ((line_point.x - point.x) * (line_point.x - point.x)) / ((line_direction.x * 2.0) * (line_direction.x * 2.0)) + (line_direction.y * 2.0) * (point.z - line_point.z) / ((line_direction.z * 2.0) * (line_direction.z * 2.0)),
        line_point.y + (line_point.y - point.y) * ((line_point.y - point.y) * (line_point.y - point.y)) / ((line_direction.y * 2.0) * (line_direction.y * 2.0)) + (line_direction.x * 2.0) * (point.x - line_point.x) / ((line_direction.x * 2.0) * (line_direction.x * 2.0)),
        line_point.z + (line_point.z - point.z) * ((line_point.z - point.z) * (line_point.z - point.z)) / ((line_direction.z * 2.0) * (line_direction.z * 2.0)) + (line_direction.y * 2.0) * (point.y - line_point.y) / ((line_direction.y * 2.0) * (line_direction.y * 2.0))};
    return v;
}

pcl::PointXYZ minus(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2)
{
    return pcl::PointXYZ{
        point1.x - point2.x,
        point1.y - point2.y,
        point1.z - point2.z};
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generate_random_pointcloud(size_t num_points, float min, float max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < num_points; i++)
    {
        pcl::PointXYZ point{
            min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min))),
            min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min))),
            min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)))};
        pointcloud->push_back(point);
    }
    return pointcloud;
}

float heavy_computing(const pcl::PointXYZ &point, unsigned int iterations)
{
    float result = distance_to_origin(point);
    for (unsigned int i = 0; i < iterations; i++)
    {
        result += dot_product(
            point,
            pcl::PointXYZ{1.0, 2.0, 3.0});
        result += cross_product(
                      point,
                      pcl::PointXYZ{4.0, 5.0, 6.0})
                      .x;
        result = result + (result * 10.0);
        if (result > 0.0)
        {
            result = std::sqrt(result);
        }
        pcl::PointXYZ reflected_point = reflection_through_plane(
            point,
            pcl::PointXYZ{7.0, 8.0, 9.0},
            pcl::PointXYZ{3.0, 4.0, 5.0});
        pcl::PointXYZ rotated_point = rotation_about_x(
            pcl::PointXYZ{10.0, 11.0, 12.0},
            3.14159);

        result += magnitude_squared(minus(reflected_point, rotated_point));
    }
    return result;
}

bool roundtrip(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCLPointCloud2 pcl_pc2; // "data blob"
    pcl::toPCLPointCloud2(cloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 sensor_pc2;
    pcl_conversions::fromPCL(pcl_pc2, sensor_pc2); // now in ROS format

    pcl::PCLPointCloud2 pcl_pc2_out;
    pcl_conversions::toPCL(sensor_pc2, pcl_pc2_out);
    pcl::fromPCLPointCloud2(pcl_pc2_out, *cloud_filtered);

    return cloud.size() == cloud_filtered->size();
}

bool roundtrip_filter(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conv(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCLPointCloud2 pcl_pc2; // "data blob"
    pcl::toPCLPointCloud2(cloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 sensor_pc2;
    pcl_conversions::fromPCL(pcl_pc2, sensor_pc2); // now in ROS format

    pcl::PCLPointCloud2 pcl_pc2_out;
    pcl_conversions::toPCL(sensor_pc2, pcl_pc2_out);
    pcl::fromPCLPointCloud2(pcl_pc2_out, *cloud_conv);

    size_t orig_len = cloud_conv->size();

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_conv);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5.0, 5.0);
    pass.filter(*cloud_filtered);

    return cloud_filtered->size() < orig_len;
}

bool roundtrip_filter_par(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conv(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCLPointCloud2 pcl_pc2; // "data blob"
    pcl::toPCLPointCloud2(cloud, pcl_pc2);
    sensor_msgs::msg::PointCloud2 sensor_pc2;
    pcl_conversions::fromPCL(pcl_pc2, sensor_pc2); // now in ROS format

    pcl::PCLPointCloud2 pcl_pc2_out;
    pcl_conversions::toPCL(sensor_pc2, pcl_pc2_out);
    pcl::fromPCLPointCloud2(pcl_pc2_out, *cloud_conv);

    size_t orig_len = cloud_conv->size();

#pragma omp parallel for
    for (size_t i = 0; i < cloud_conv->size(); i++)
    {
        if (cloud_conv->points[i].x >= -5.0 && cloud_conv->points[i].x <= 5.0)
        {
#pragma omp critical
            cloud_filtered->push_back(cloud_conv->points[i]);
        }
    }

    return cloud_filtered->size() < orig_len;
}

bool roundtrip_computing(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    float total = 0.0;
    for (const pcl::PointXYZ &point : cloud.points)
    {
        total += heavy_computing(point, 100); // already inlined to the iteration compared to .sum() in Rust
    }
    return total > 0.0;
}

bool roundtrip_computing_par(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    float total = 0.0;

#pragma omp parallel for reduction(+ : total)
    for (size_t i = 0; i < cloud.size(); i++)
    {
        total += heavy_computing(cloud.points[i], 100); // already inlined to the iteration compared to .sum() in Rust
    }
    return total > 0.0;
}