#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include <limits>

#include "processing.hpp"

int main(int argc, char *argv[])
{
  auto random_cpl = generate_random_pointcloud(16000, -10000.0, 10000.0);

  int num_roundtrips = 10;
  for (int i = 0; i < num_roundtrips; i++)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_copy = *random_cpl;
    if (!roundtrip_computing(cloud_copy))
    {
      return 1;
    }
  }

  return 0;
}