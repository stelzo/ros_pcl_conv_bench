# `ros_pointcloud2` vs `PCL` Benchmark

This repository presents a benchmark suite for comparing ROS conversions to and from the PointCloud2 message in common scenarios.

## Running it locally

Hyperfine is used as benchmarking tool. You also must be in a sourced ROS2 bash and have Rust installed.

```shell
sudo apt install hyperfine
```

For working easily with all the binaries, that each showcase a specific use case, we use Make.

```shell
make run
```

The results are saved as Markdown and json files in `./results`.
You can plot them with `python3 plot.py`.

Run `make clean` for deleting all generated files.

The following measurements are from a NVIDIA Jetson Orin AGX with JP6 and auto generated with output of `hyperfine`.

# Results

| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_120k_pcl` | 832.0 ± 13.8 | 812.7 | 851.5 | 3.46 ± 0.07 |
| `./rpcl2/target/release/compute_par_par_120k` | 240.1 ± 2.9 | 235.8 | 245.0 | 1.00 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_60k_pcl` | 2.869 ± 0.003 | 2.865 | 2.875 | 23.02 ± 0.46 |
| `./build/pcl/compute_par_60k_pcl` | 0.435 ± 0.007 | 0.425 | 0.449 | 3.49 ± 0.09 |
| `./rpcl2/target/release/compute_par_par_60k` | 0.125 ± 0.002 | 0.122 | 0.133 | 1.00 |
| `./rpcl2/target/release/compute_60k` | 0.842 ± 0.001 | 0.842 | 0.844 | 6.76 ± 0.13 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_500k_pcl` | 3.436 ± 0.015 | 3.415 | 3.455 | 3.61 ± 0.02 |
| `./rpcl2/target/release/compute_par_par_500k` | 0.952 ± 0.004 | 0.945 | 0.960 | 1.00 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_500k_pcl` | 202.9 ± 3.0 | 199.0 | 208.1 | 2.62 ± 0.07 |
| `./rpcl2/target/release/roundtrip_vec_500k` | 77.6 ± 1.8 | 75.8 | 83.3 | 1.00 |
| `./rpcl2/target/release/roundtrip_500k` | 354.8 ± 5.2 | 349.2 | 365.0 | 4.57 ± 0.13 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_16k_pcl` | 33.5 ± 0.3 | 32.8 | 34.5 | 7.99 ± 0.21 |
| `./rpcl2/target/release/roundtrip_vec_16k` | 4.2 ± 0.1 | 4.0 | 4.6 | 1.00 |
| `./rpcl2/target/release/roundtrip_16k` | 13.0 ± 0.1 | 12.8 | 14.1 | 3.10 ± 0.08 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_1_5m_pcl` | 681.6 ± 5.1 | 676.6 | 694.2 | 2.23 ± 0.04 |
| `./build/pcl/filter_par_1_5m_pcl` | 419.4 ± 9.9 | 405.0 | 433.9 | 1.37 ± 0.04 |
| `./rpcl2/target/release/filter_par_par_1_5m` | 305.6 ± 5.1 | 298.8 | 312.0 | 1.00 |
| `./rpcl2/target/release/filter_vec_1_5m` | 916.6 ± 5.8 | 909.7 | 924.8 | 3.00 ± 0.05 |
| `./rpcl2/target/release/filter_1_5m` | 1120.1 ± 15.8 | 1102.2 | 1144.9 | 3.67 ± 0.08 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_60k_pcl` | 64.8 ± 0.4 | 63.9 | 66.1 | 2.72 ± 0.19 |
| `./build/pcl/filter_par_60k_pcl` | 48.5 ± 1.6 | 46.1 | 54.3 | 2.04 ± 0.15 |
| `./rpcl2/target/release/filter_par_par_60k` | 23.8 ± 1.6 | 20.3 | 29.6 | 1.00 |
| `./rpcl2/target/release/filter_vec_60k` | 36.4 ± 0.9 | 35.6 | 42.9 | 1.53 ± 0.11 |
| `./rpcl2/target/release/filter_60k` | 43.4 ± 0.8 | 42.7 | 49.6 | 1.82 ± 0.13 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_16k_pcl` | 36.4 ± 0.4 | 35.8 | 38.4 | 3.45 ± 0.31 |
| `./build/pcl/filter_par_16k_pcl` | 34.2 ± 1.5 | 32.4 | 38.3 | 3.25 ± 0.32 |
| `./rpcl2/target/release/filter_par_par_16k` | 10.5 ± 0.9 | 9.4 | 14.7 | 1.00 |
| `./rpcl2/target/release/filter_vec_16k` | 11.0 ± 0.4 | 10.6 | 12.9 | 1.05 ± 0.10 |
| `./rpcl2/target/release/filter_16k` | 12.9 ± 0.2 | 12.5 | 14.5 | 1.22 ± 0.11 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_1_5m_pcl` | 488.2 ± 5.1 | 477.6 | 493.7 | 2.32 ± 0.04 |
| `./rpcl2/target/release/roundtrip_vec_1_5m` | 210.7 ± 2.5 | 202.5 | 212.9 | 1.00 |
| `./rpcl2/target/release/roundtrip_1_5m` | 1077.8 ± 8.7 | 1064.8 | 1089.8 | 5.12 ± 0.07 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_120k_pcl` | 90.7 ± 0.5 | 90.0 | 92.3 | 3.67 ± 0.05 |
| `./rpcl2/target/release/roundtrip_vec_120k` | 24.7 ± 0.3 | 24.1 | 25.5 | 1.00 |
| `./rpcl2/target/release/roundtrip_120k` | 91.2 ± 0.5 | 90.6 | 92.5 | 3.69 ± 0.05 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_1_5m_pcl` | 10.243 ± 0.048 | 10.143 | 10.294 | 3.63 ± 0.02 |
| `./rpcl2/target/release/compute_par_par_1_5m` | 2.824 ± 0.007 | 2.816 | 2.843 | 1.00 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_500k_pcl` | 274.4 ± 3.6 | 269.4 | 280.0 | 2.35 ± 0.07 |
| `./build/pcl/filter_par_500k_pcl` | 171.9 ± 4.9 | 162.6 | 182.7 | 1.47 ± 0.06 |
| `./rpcl2/target/release/filter_par_par_500k` | 116.8 ± 3.1 | 110.8 | 123.7 | 1.00 |
| `./rpcl2/target/release/filter_vec_500k` | 322.1 ± 3.6 | 316.3 | 329.0 | 2.76 ± 0.08 |
| `./rpcl2/target/release/filter_500k` | 394.9 ± 11.7 | 382.5 | 413.4 | 3.38 ± 0.13 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_120k_pcl` | 109.3 ± 0.6 | 108.1 | 110.6 | 4.61 ± 0.34 |
| `./build/pcl/filter_par_120k_pcl` | 71.7 ± 3.8 | 67.0 | 83.4 | 3.02 ± 0.27 |
| `./rpcl2/target/release/filter_par_par_120k` | 23.7 ± 1.7 | 19.3 | 30.6 | 1.00 |
| `./rpcl2/target/release/filter_vec_120k` | 74.5 ± 5.8 | 70.5 | 86.2 | 3.14 ± 0.34 |
| `./rpcl2/target/release/filter_120k` | 89.8 ± 7.4 | 84.1 | 112.2 | 3.78 ± 0.42 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_16k_pcl` | 786.9 ± 0.9 | 785.3 | 788.2 | 20.59 ± 1.27 |
| `./build/pcl/compute_par_16k_pcl` | 137.0 ± 7.7 | 130.9 | 157.3 | 3.59 ± 0.30 |
| `./rpcl2/target/release/compute_par_par_16k` | 38.2 ± 2.4 | 35.5 | 48.0 | 1.00 |
| `./rpcl2/target/release/compute_16k` | 225.6 ± 0.2 | 225.4 | 226.1 | 5.90 ± 0.37 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_60k_pcl` | 55.5 ± 0.3 | 54.7 | 56.4 | 4.57 ± 0.09 |
| `./rpcl2/target/release/roundtrip_vec_60k` | 12.1 ± 0.2 | 11.8 | 13.4 | 1.00 |
| `./rpcl2/target/release/roundtrip_60k` | 45.7 ± 0.4 | 45.1 | 46.9 | 3.76 ± 0.08 |
