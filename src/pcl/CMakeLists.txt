cmake_minimum_required(VERSION 3.5)
project(pcl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(roundtrip_1_5m_pcl src/roundtrip_1_5m.cpp)
add_executable(roundtrip_500k_pcl src/roundtrip_500k.cpp)
add_executable(roundtrip_120k_pcl src/roundtrip_120k.cpp)
add_executable(roundtrip_60k_pcl src/roundtrip_60k.cpp)
add_executable(roundtrip_16k_pcl src/roundtrip_16k.cpp)
add_executable(compute_60k_pcl src/compute_60k.cpp)
add_executable(compute_16k_pcl src/compute_16k.cpp)
add_executable(compute_par_60k_pcl src/compute_par_60k.cpp)
add_executable(compute_par_16k_pcl src/compute_par_16k.cpp)
add_executable(compute_120k_pcl src/compute_120k.cpp)
add_executable(compute_par_120k_pcl src/compute_par_120k.cpp)
add_executable(compute_500k_pcl src/compute_500k.cpp)
add_executable(compute_par_500k_pcl src/compute_par_500k.cpp)
add_executable(compute_1_5m_pcl src/compute_1_5m.cpp)
add_executable(compute_par_1_5m_pcl src/compute_par_1_5m.cpp)
add_executable(filter_60k_pcl src/filter_60k.cpp)
add_executable(filter_120k_pcl src/filter_120k.cpp)
add_executable(filter_16k_pcl src/filter_16k.cpp)
add_executable(filter_par_60k_pcl src/filter_par_60k.cpp)
add_executable(filter_par_120k_pcl src/filter_par_120k.cpp)
add_executable(filter_par_16k_pcl src/filter_par_16k.cpp)
add_executable(filter_500k_pcl src/filter_500k.cpp)
add_executable(filter_par_500k_pcl src/filter_par_500k.cpp)
add_executable(filter_par_1_5m_pcl src/filter_par_1_5m.cpp)
add_executable(filter_1_5m_pcl src/filter_1_5m.cpp)

target_link_libraries( filter_60k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( filter_120k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( filter_500k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( filter_1_5m_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( filter_16k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_60k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_16k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_par_60k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( compute_par_16k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( compute_120k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_par_120k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( compute_500k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_par_500k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( compute_1_5m_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( compute_par_1_5m_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( roundtrip_60k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( roundtrip_16k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( roundtrip_120k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries( roundtrip_500k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)
target_link_libraries( roundtrip_1_5m_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries( filter_par_60k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( filter_par_120k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( filter_par_1_5m_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( filter_par_500k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

target_link_libraries( filter_par_16k_pcl
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(
  filter_par_500k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_500k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_par_1_5m_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_1_5m_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_par_60k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_par_120k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)


ament_target_dependencies(
  filter_par_16k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_60k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_120k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  filter_16k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  roundtrip_60k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  roundtrip_16k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  roundtrip_120k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)
ament_target_dependencies(
  roundtrip_500k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  roundtrip_1_5m_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_60k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_16k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_par_16k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_par_60k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_120k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_par_120k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_500k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_par_500k_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)


ament_target_dependencies(
  compute_1_5m_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(
  compute_par_1_5m_pcl
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

ament_package()
