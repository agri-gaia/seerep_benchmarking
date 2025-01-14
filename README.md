# SEEREP Benchmarking

Performance benchmark comparing the HDF5 ROS interface of SEEREP with MCAP.

## Table of Contents

- [Description](#description)
- [Settings](#settings)
- [Installation](#installation)
  - [Docker](#docker)
  - [Locally](#locally)
- [Running the Benchmark](#running-the-benchmark)
- [Reference](#reference)
- [TODO](#todo)

## Description

To evaluate the performance of the HDF5 ROS interface of SEEREP,
a benchmark on maximal data throughput is conducted.
This is an important characteristic, as it determines the number of
concurrent sensor streams that can be handled. For comparison [MCAP](https://mcap.dev/),
the new default recording format in ROS 2, is used.

The general procedure involves sampling ROS messages of different sizes from
a provided file. For instance, for 10 KiB messages, the first message
would contain the range $[0,10)$ KiB, while the second would cover $[10,20)$ KiB.
The messages are sampled until a certain amount of total data is reached.
When the total size is not a multiple of the message size, the last message is
limited to the remaining size.

For both HDF5 and MCAP, the time for each write call is measured without including
setup tasks such as serialization or file opening and closing.
For MCAP the messages are stored using the C++ open source
[implementation](https://github.com/foxglove/mcap/tree/main/cpp).
Since MCAP depends on external serialization, the benchmark uses ROS 1
serialization for simplicity. For HDF5 the [HighFive](https://github.com/BlueBrain/HighFive)
header only library is used. The message payloads are stored as a contiguous
1D dataset, while other fields are stored as attributes to the dataset.
For SEEREP's use case, storing the images as a contiguous dataset is
reasonable because the data is fully read when the images match a specific query.

## Installation

Clone the repository with submodules:

```bash
git clone --recurse-submodules git@github.com:agri-gaia/seerep_benchmarking.git
```

### Docker

The provided Dockerfile includes all dependencies to run the benchmark. It is
based on the SEEREP
[base image](https://github.com/agri-gaia/seerep/pkgs/container/seerep_base).
To build the image use:

```bash
docker build -t seerep-benchmarking .
```

### Locally

To eliminate any influence from Docker, the benchmark can also be run locally with some additional effort. For that install:

- ROS Noetic (**requires Ubuntu 20.04!**)
- Catkin
- HighFive
- Boost
- Protocol Buffers
- Conan, Pandas

Use the SEEREP base [Dockerfile](https://github.com/agri-gaia/seerep/blob/main/docker/base/Dockerfile)
and the local [Dockerfile](./Dockerfile) as a reference.

Also make sure to build with optimization:

```bash
catkin build seerep_benchmarking -DCMAKE_BUILD_TYPE=Release
```

## Settings

Currently the settings for the benchmark can be changed in the
[`config.json`](./seerep_benchmarking/config/config.json) file. The following settings are
available:

| Parameter | Description |
|----------|----------|
| host_dir | Path to store the results of the benchmark (figures, csv, HDF5 and MCAP files)  |
| payload_file_name  | File to use for message payload (has to be located in the `host_dir`!)  |
| num_runs  | Total number of runs for each message size   |
| message_sizes | Array of message sizes (in bytes) to test  |
| total_sizes  | Array of total data size (in bytes) to use for each run  |

To change the storage options in MCAP and HDF5, the corresponding C++ code has to be adjusted.
For MCAP add or remove setting from the `mcap::McapWriter`
[here](https://github.com/agri-gaia/seerep_benchmarking/blob/251cb0f7d9d30bfff530f3f8a7cb8abffda7fb10/seerep_benchmarking/src/analysis.cpp#L47-L53), for HDF5 different
[properties](https://bluebrain.github.io/HighFive/group___property_lists.html#ga8877fba7ca191b3b155888a625a764c3)
would have to be set in the `seerep_hdf5_ros` package.

## Running the MCAP-HDF5-Write-Benchmark

In the Docker setup, a bind mount is used for I/O with the host file system.
To run the previously built image, use:

```bash
docker run \
    -v {/host/dir}:/home/docker/host_dir \
    -it seerep-benchmarking 
```

> [!WARNING]  
> Using bind mounts on Windows or MacOS is known to result in slower disk performance.
> For more information refer to [this article](https://code.visualstudio.com/remote/advancedcontainers/improve-performance).
> Using named volumes should resolve this issue, though it requires copying data in and out of the
> volume using `docker cp`.

To run the benchmark use:

```bash
rosrun seerep_benchmarking run.py 
```

The output should look like:

```bash
Running 10KiB-250MiB ...
Run 1 ...
Run 2 ...
Run 3 ...
...
```

The resulting csv data and plots will be place in the host directory.

To plot already present results, for example after changing some figure settings use:

```bash
rosrun seerep_benchmarking run.py --plot-only
```

## Running the rosbag-HDF5-converter-Benchmark
To run this benchmark you can also use the docker container as described in the
previous section. After building the code locally or when using the container the
converter can be started with the following command:

```bash
roslaunch seerep_benchmarking rosbag-HDF5-converter.launch
```

The configuration of the converter benchmark can be changed in `seerep_benchmarking/launch/rosbag-HDF5-converter.launch`.

The results will be printed to the terminal and can be found between the ros outputs.
They look like this:

```bash
mean: 5599879736 ns
mean: 5.59988 s
standard deviation: 1385599923 ns
standard deviation: 1385.6 ms
```


## Reference

For further details, refer to the related publication at ICRA 2024:

```bash
@inproceedings{Niemeyer2024,
  author = {Niemeyer, Mark and Arkenau, Julian and Pütz, Sebastian and
  Hertzberg, Joachim},
  title = {Streamlined Acquisition of Large Sensor Data for Autonomous Mobile
  Robots to   Enable Efficient Creation and Analysis of Datasets },
  booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year = {2024},
  publisher = {IEEE}
}
```

## TODO

- [ ] Merge up to date version of `seerep_hdf5_ros` into SEEREP main branch
