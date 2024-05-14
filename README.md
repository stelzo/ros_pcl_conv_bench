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

| Command                                      |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------- | ----------: | -------: | -------: | ----------: |
| `./build/pcl/filter_500k_pcl`                | 161.6 ± 2.3 |    158.9 |    165.7 | 3.65 ± 0.57 |
| `./build/pcl/filter_par_500k_pcl`            | 157.3 ± 7.9 |    141.7 |    169.9 | 3.56 ± 0.58 |
| `./rpcl2/target/release/filter_par_par_500k` |  44.2 ± 6.8 |     35.5 |     56.5 |        1.00 |
| `./rpcl2/target/release/filter_vec_500k`     | 123.3 ± 1.4 |    120.9 |    126.5 | 2.79 ± 0.43 |
| `./rpcl2/target/release/filter_500k`         | 142.6 ± 2.2 |    139.3 |    145.4 | 3.22 ± 0.50 |

| Command                                       |      Mean [s] | Min [s] | Max [s] |    Relative |
| :-------------------------------------------- | ------------: | ------: | ------: | ----------: |
| `./build/pcl/compute_par_500k_pcl`            | 1.295 ± 0.038 |   1.234 |   1.340 | 8.02 ± 0.30 |
| `./rpcl2/target/release/compute_par_par_500k` | 0.161 ± 0.004 |   0.157 |   0.175 |        1.00 |

| Command                                      |    Mean [ms] | Min [ms] | Max [ms] |     Relative |
| :------------------------------------------- | -----------: | -------: | -------: | -----------: |
| `./build/pcl/compute_16k_pcl`                |  515.3 ± 2.9 |    511.2 |    521.1 | 55.59 ± 3.68 |
| `./build/pcl/compute_par_16k_pcl`            | 141.7 ± 19.6 |     63.5 |    157.1 | 15.28 ± 2.34 |
| `./rpcl2/target/release/compute_par_par_16k` |    9.3 ± 0.6 |      8.2 |     13.1 |         1.00 |
| `./rpcl2/target/release/compute_16k`         |   91.8 ± 0.6 |     91.0 |     93.2 |  9.90 ± 0.66 |

| Command                                    |  Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :----------------------------------------- | ---------: | -------: | -------: | ----------: |
| `./build/pcl/roundtrip_60k_pcl`            | 17.3 ± 0.4 |     16.5 |     19.1 | 4.47 ± 0.21 |
| `./rpcl2/target/release/roundtrip_vec_60k` |  3.9 ± 0.2 |      3.5 |      5.2 |        1.00 |
| `./rpcl2/target/release/roundtrip_60k`     | 17.5 ± 0.9 |     16.7 |     27.2 | 4.52 ± 0.30 |

| Command                                     |  Mean [ms] | Min [ms] | Max [ms] |     Relative |
| :------------------------------------------ | ---------: | -------: | -------: | -----------: |
| `./build/pcl/filter_16k_pcl`                |  9.3 ± 0.3 |      8.5 |     10.0 |  2.99 ± 0.24 |
| `./build/pcl/filter_par_16k_pcl`            | 97.2 ± 6.4 |     85.1 |    121.8 | 31.32 ± 3.15 |
| `./rpcl2/target/release/filter_par_par_16k` |  6.2 ± 0.7 |      5.1 |     10.3 |  2.01 ± 0.27 |
| `./rpcl2/target/release/filter_vec_16k`     |  3.1 ± 0.2 |      2.8 |      4.8 |         1.00 |
| `./rpcl2/target/release/filter_16k`         |  3.8 ± 0.2 |      3.5 |      5.6 |  1.23 ± 0.11 |

| Command                                      |    Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------- | -----------: | -------: | -------: | ----------: |
| `./build/pcl/filter_1_5m_pcl`                |  497.6 ± 2.6 |    494.6 |    502.5 | 4.50 ± 0.41 |
| `./build/pcl/filter_par_1_5m_pcl`            | 349.8 ± 28.2 |    304.0 |    400.0 | 3.16 ± 0.38 |
| `./rpcl2/target/release/filter_par_par_1_5m` | 110.7 ± 10.0 |    100.9 |    151.5 |        1.00 |
| `./rpcl2/target/release/filter_vec_1_5m`     |  376.8 ± 4.5 |    371.6 |    386.3 | 3.41 ± 0.31 |
| `./rpcl2/target/release/filter_1_5m`         |  443.5 ± 3.6 |    437.7 |    449.3 | 4.01 ± 0.37 |

| Command                                    | Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :----------------------------------------- | --------: | -------: | -------: | ----------: |
| `./build/pcl/roundtrip_16k_pcl`            | 8.1 ± 0.4 |      7.3 |     12.6 | 6.97 ± 0.62 |
| `./rpcl2/target/release/roundtrip_vec_16k` | 1.2 ± 0.1 |      1.0 |      2.5 |        1.00 |
| `./rpcl2/target/release/roundtrip_16k`     | 4.9 ± 0.2 |      4.5 |      7.3 | 4.20 ± 0.36 |

| Command                                     |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------ | ----------: | -------: | -------: | ----------: |
| `./build/pcl/roundtrip_1_5m_pcl`            | 405.6 ± 4.9 |    400.3 |    414.7 | 3.12 ± 0.05 |
| `./rpcl2/target/release/roundtrip_vec_1_5m` | 129.9 ± 1.3 |    127.6 |    132.3 |        1.00 |
| `./rpcl2/target/release/roundtrip_1_5m`     | 561.2 ± 8.4 |    551.8 |    576.6 | 4.32 ± 0.08 |

| Command                                     |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------ | ----------: | -------: | -------: | ----------: |
| `./build/pcl/filter_60k_pcl`                |  21.3 ± 0.9 |     20.0 |     29.3 | 2.09 ± 0.11 |
| `./build/pcl/filter_par_60k_pcl`            | 100.5 ± 5.0 |     92.9 |    118.0 | 9.86 ± 0.56 |
| `./rpcl2/target/release/filter_par_par_60k` |  11.7 ± 1.6 |      9.8 |     30.3 | 1.15 ± 0.16 |
| `./rpcl2/target/release/filter_vec_60k`     |  10.2 ± 0.3 |      9.8 |     12.1 |        1.00 |
| `./rpcl2/target/release/filter_60k`         |  13.0 ± 0.9 |     12.5 |     24.1 | 1.28 ± 0.10 |

| Command                                       |      Mean [s] | Min [s] | Max [s] |    Relative |
| :-------------------------------------------- | ------------: | ------: | ------: | ----------: |
| `./build/pcl/compute_par_1_5m_pcl`            | 3.602 ± 0.062 |   3.517 |   3.707 | 7.94 ± 0.15 |
| `./rpcl2/target/release/compute_par_par_1_5m` | 0.454 ± 0.004 |   0.446 |   0.459 |        1.00 |

| Command                                      |      Mean [s] | Min [s] | Max [s] |     Relative |
| :------------------------------------------- | ------------: | ------: | ------: | -----------: |
| `./build/pcl/compute_60k_pcl`                | 1.899 ± 0.006 |   1.891 |   1.910 | 80.53 ± 2.98 |
| `./build/pcl/compute_par_60k_pcl`            | 0.277 ± 0.010 |   0.263 |   0.291 | 11.74 ± 0.61 |
| `./rpcl2/target/release/compute_par_par_60k` | 0.024 ± 0.001 |   0.022 |   0.026 |         1.00 |
| `./rpcl2/target/release/compute_60k`         | 0.345 ± 0.002 |   0.341 |   0.347 | 14.63 ± 0.55 |

| Command                                     |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------ | ----------: | -------: | -------: | ----------: |
| `./build/pcl/roundtrip_500k_pcl`            | 129.3 ± 1.6 |    127.0 |    133.5 | 3.21 ± 0.08 |
| `./rpcl2/target/release/roundtrip_vec_500k` |  40.3 ± 0.9 |     39.2 |     45.1 |        1.00 |
| `./rpcl2/target/release/roundtrip_500k`     | 155.1 ± 1.5 |    152.4 |    157.9 | 3.85 ± 0.10 |

| Command                                       |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :-------------------------------------------- | ----------: | -------: | -------: | ----------: |
| `./build/pcl/compute_par_120k_pcl`            | 398.9 ± 5.5 |    388.5 |    406.3 | 9.56 ± 0.45 |
| `./rpcl2/target/release/compute_par_par_120k` |  41.7 ± 1.9 |     38.3 |     51.5 |        1.00 |

| Command                                     |  Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------ | ---------: | -------: | -------: | ----------: |
| `./build/pcl/roundtrip_120k_pcl`            | 32.5 ± 0.6 |     31.3 |     34.7 | 4.02 ± 0.14 |
| `./rpcl2/target/release/roundtrip_vec_120k` |  8.1 ± 0.2 |      7.4 |     10.6 |        1.00 |
| `./rpcl2/target/release/roundtrip_120k`     | 34.9 ± 1.4 |     34.0 |     43.5 | 4.31 ± 0.21 |

| Command                                      |   Mean [ms] | Min [ms] | Max [ms] |    Relative |
| :------------------------------------------- | ----------: | -------: | -------: | ----------: |
| `./build/pcl/filter_120k_pcl`                |  40.2 ± 1.5 |     38.5 |     49.1 | 3.53 ± 0.45 |
| `./build/pcl/filter_par_120k_pcl`            | 107.5 ± 5.2 |    101.3 |    121.9 | 9.43 ± 1.24 |
| `./rpcl2/target/release/filter_par_par_120k` |  11.4 ± 1.4 |      9.2 |     23.9 |        1.00 |
| `./rpcl2/target/release/filter_vec_120k`     |  22.2 ± 2.9 |     19.7 |     28.7 | 1.95 ± 0.35 |
| `./rpcl2/target/release/filter_120k`         |  27.0 ± 3.2 |     24.8 |     44.7 | 2.37 ± 0.41 |
