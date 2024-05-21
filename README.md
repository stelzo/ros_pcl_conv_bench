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

The following measurements are from an Intel i7-14700 and auto generated with output of `hyperfine`.

# Results

| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_500k_pcl` | 167.4 ± 3.1 | 162.1 | 174.9 | 2.43 ± 0.15 |
| `./build/pcl/filter_par_500k_pcl` | 147.3 ± 11.2 | 128.9 | 172.0 | 2.14 ± 0.20 |
| `./rpcl2/target/release/filter_par_par_500k` | 68.9 ± 4.0 | 60.5 | 77.1 | 1.00 |
| `./rpcl2/target/release/filter_vec_500k` | 198.1 ± 0.8 | 197.1 | 199.8 | 2.87 ± 0.17 |
| `./rpcl2/target/release/filter_500k` | 230.2 ± 1.3 | 227.3 | 232.5 | 3.34 ± 0.20 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_500k_pcl` | 1.296 ± 0.023 | 1.243 | 1.326 | 6.60 ± 0.19 |
| `./rpcl2/target/release/compute_par_par_500k` | 0.196 ± 0.004 | 0.186 | 0.203 | 1.00 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_16k_pcl` | 516.8 ± 1.5 | 515.5 | 520.7 | 46.01 ± 2.56 |
| `./build/pcl/compute_par_16k_pcl` | 142.6 ± 12.6 | 106.1 | 152.9 | 12.70 ± 1.33 |
| `./rpcl2/target/release/compute_par_par_16k` | 11.2 ± 0.6 | 9.4 | 14.5 | 1.00 |
| `./rpcl2/target/release/compute_16k` | 95.9 ± 0.4 | 94.7 | 96.7 | 8.54 ± 0.48 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_60k_pcl` | 17.1 ± 0.7 | 15.8 | 24.2 | 2.88 ± 0.69 |
| `./rpcl2/target/release/roundtrip_vec_60k` | 5.9 ± 1.4 | 5.2 | 19.8 | 1.00 |
| `./rpcl2/target/release/roundtrip_60k` | 22.4 ± 0.3 | 21.9 | 24.2 | 3.78 ± 0.89 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_16k_pcl` | 9.0 ± 0.3 | 8.4 | 10.9 | 1.79 ± 0.31 |
| `./build/pcl/filter_par_16k_pcl` | 36.1 ± 40.5 | 7.0 | 97.8 | 7.16 ± 8.12 |
| `./rpcl2/target/release/filter_par_par_16k` | 8.0 ± 0.7 | 6.6 | 13.6 | 1.59 ± 0.31 |
| `./rpcl2/target/release/filter_vec_16k` | 5.0 ± 0.9 | 4.6 | 16.8 | 1.00 |
| `./rpcl2/target/release/filter_16k` | 7.1 ± 0.7 | 5.9 | 16.5 | 1.40 ± 0.28 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_1_5m_pcl` | 506.0 ± 7.6 | 497.6 | 518.6 | 2.51 ± 0.12 |
| `./build/pcl/filter_par_1_5m_pcl` | 346.2 ± 30.0 | 306.2 | 405.4 | 1.72 ± 0.17 |
| `./rpcl2/target/release/filter_par_par_1_5m` | 201.3 ± 9.2 | 175.4 | 211.3 | 1.00 |
| `./rpcl2/target/release/filter_vec_1_5m` | 600.3 ± 3.6 | 594.1 | 605.3 | 2.98 ± 0.14 |
| `./rpcl2/target/release/filter_1_5m` | 703.7 ± 36.5 | 687.6 | 807.2 | 3.50 ± 0.24 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_16k_pcl` | 8.0 ± 0.4 | 7.5 | 13.5 | 5.04 ± 2.06 |
| `./rpcl2/target/release/roundtrip_vec_16k` | 1.6 ± 0.6 | 1.4 | 8.6 | 1.00 |
| `./rpcl2/target/release/roundtrip_16k` | 6.6 ± 0.2 | 6.3 | 7.9 | 4.16 ± 1.69 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_1_5m_pcl` | 405.2 ± 4.6 | 400.8 | 416.7 | 2.19 ± 0.03 |
| `./rpcl2/target/release/roundtrip_vec_1_5m` | 184.7 ± 1.8 | 181.4 | 187.7 | 1.00 |
| `./rpcl2/target/release/roundtrip_1_5m` | 731.8 ± 2.8 | 726.7 | 735.4 | 3.96 ± 0.04 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_60k_pcl` | 21.1 ± 0.4 | 19.9 | 22.3 | 1.41 ± 0.18 |
| `./build/pcl/filter_par_60k_pcl` | 55.0 ± 33.2 | 12.9 | 135.6 | 3.69 ± 2.27 |
| `./rpcl2/target/release/filter_par_par_60k` | 14.9 ± 1.9 | 12.6 | 25.2 | 1.00 |
| `./rpcl2/target/release/filter_vec_60k` | 17.5 ± 0.5 | 17.0 | 20.9 | 1.17 ± 0.15 |
| `./rpcl2/target/release/filter_60k` | 26.3 ± 0.7 | 25.3 | 30.6 | 1.77 ± 0.23 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_1_5m_pcl` | 3.614 ± 0.061 | 3.505 | 3.711 | 6.39 ± 0.18 |
| `./rpcl2/target/release/compute_par_par_1_5m` | 0.565 ± 0.013 | 0.536 | 0.579 | 1.00 |


| Command | Mean [s] | Min [s] | Max [s] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_60k_pcl` | 1.922 ± 0.006 | 1.915 | 1.936 | 69.44 ± 2.54 |
| `./build/pcl/compute_par_60k_pcl` | 0.267 ± 0.026 | 0.197 | 0.293 | 9.66 ± 0.99 |
| `./rpcl2/target/release/compute_par_par_60k` | 0.028 ± 0.001 | 0.025 | 0.030 | 1.00 |
| `./rpcl2/target/release/compute_60k` | 0.357 ± 0.001 | 0.355 | 0.358 | 12.90 ± 0.47 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_500k_pcl` | 133.1 ± 1.2 | 131.0 | 135.7 | 2.27 ± 0.04 |
| `./rpcl2/target/release/roundtrip_vec_500k` | 58.6 ± 0.7 | 57.5 | 60.8 | 1.00 |
| `./rpcl2/target/release/roundtrip_500k` | 211.7 ± 1.9 | 208.9 | 215.4 | 3.61 ± 0.06 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/compute_par_120k_pcl` | 396.3 ± 15.0 | 366.9 | 409.5 | 7.82 ± 0.40 |
| `./rpcl2/target/release/compute_par_par_120k` | 50.7 ± 1.7 | 46.3 | 55.2 | 1.00 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/roundtrip_120k_pcl` | 32.9 ± 1.1 | 30.2 | 37.2 | 2.46 ± 0.10 |
| `./rpcl2/target/release/roundtrip_vec_120k` | 13.4 ± 0.3 | 12.6 | 16.5 | 1.00 |
| `./rpcl2/target/release/roundtrip_120k` | 48.9 ± 0.7 | 47.7 | 52.5 | 3.66 ± 0.11 |


| Command | Mean [ms] | Min [ms] | Max [ms] | Relative |
|:---|---:|---:|---:|---:|
| `./build/pcl/filter_120k_pcl` | 40.6 ± 0.8 | 39.1 | 44.6 | 2.73 ± 0.22 |
| `./build/pcl/filter_par_120k_pcl` | 107.1 ± 4.2 | 100.3 | 115.3 | 7.22 ± 0.64 |
| `./rpcl2/target/release/filter_par_par_120k` | 14.8 ± 1.2 | 13.3 | 20.4 | 1.00 |
| `./rpcl2/target/release/filter_vec_120k` | 38.9 ± 4.5 | 35.5 | 55.4 | 2.62 ± 0.37 |
| `./rpcl2/target/release/filter_120k` | 53.3 ± 0.6 | 52.5 | 56.8 | 3.59 ± 0.29 |
