# gz-sim-yarp-plugins

> [!WARNING]  
> This repository contains a preliminary work in progress of integration of Modern Gazebo (gz) and YARP devices, a port of some functionalities of <https://github.com/robotology/gazebo-yarp-plugins> to Modern Gazebo. \
> The repo is working in progress, and public interfaces can change without warning.

## Table of contents

- [Installation](#installation)
- [Usage](#usage)
- [Run Tests](#run-tests)
- [Contributing](#contributing)
- [Maintainers](#maintainers)

## Installation

### conda (recommended)

You can easily install the C++ and Python library with via [`conda-forge`](https://conda-forge.org) using the following command:

```bash
conda install -c conda-forge gz-sim-yarp-plugins
```

If you are not familiar with conda or conda-forge, you can read an introduction document in [conda-forge overview](https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#conda-forge-overview).

### robotology-superbuild (advanced)

You may want to install `gz-sim-yarp-plugins` through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), an easy way to download, compile and install the robotology software on multiple operating systems, using the [CMake](https://www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). To get `gz-sim-yarp-plugins` when using the `robotology-superbuild`, please enable the `ROBOTOLOGY_USES_GZ_SIM` CMake option of the superbuild.

### Build from source (advanced)

If you want to build `gz-sim-yarp-plugins` directly from source, you can check the documentation in [`docs/build-from-source.md`](docs/build-from-source.md).

## Usage

Once the plugins are available, you can see how to use the different plugins by looking in the directories contained in the [tutorial](tutorial/) folder of this repo. Each directory is an example, and contains a README that shows how to run that example.

### Migrating from Gazebo Classic and `gazebo-yarp-plugins`

If you are migrating from an existing project made for Gazebo Classic and `gazebo-yarp-plugins`, check out the [migration guide](docs/how-to-migrate-from-gazebo-classic.md).

### How to specify Yarp configurations

There are two ways to specify the Yarp configuration of a plugin:

- `yarpConfigurationString`: it allows to directly specify the configuration in a string that must follow the [standard data representation format](https://www.yarp.it/latest/data_rep.html);
- `yarpConfigurationFile`: it specifies the location of a [Yarp configuration file](https://www.yarp.it/latest/yarp_config_files.html).

Concerning `yarpConfigurationFile`, the preferred way to specify the path to the configuration file is by using [Gazebo URIs](https://gazebosim.org/api/common/6/classgz_1_1common_1_1URI.html). A URI has the following general scheme:

```
scheme:[//authority]path[?query][#fragment] 
```

For example, the configuration file for an IMU plugin can be specified as:

```xml
<plugin name="gzyarp::Imu" filename="gz-sim-yarp-imu-system">
    <yarpConfigurationFile>model://ergoCub/conf/imu.ini</yarpConfigurationFile>
</plugin>
```

This means that the `imu.ini` file location is relative to the `ergoCub` model path, that is automatically found by Gazebo through the `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable:

```
models
    └── ergoCub
        ├── conf
        │   ├── controlboard.ini
        │   ├── ft.ini
        │   └── imu.ini
        └── model.sdf
```

> [!WARNING]
> It is possible, but strongly discouraged, to specify the location of a `yarpConfigurationFile` using absolute paths or paths relative to the current working directory: the former approach is not portable, while the latter only works if the library is loaded from the directory it was intended to be loaded from.

## Run Tests

To run the tests, just configure the project with the `BUILD_TESTING` option, and run `ctest`:

~~~
cd build
cmake -DBUILD_TESTING:BOOL=ON
ctest
~~~

For more details, check how the tests are run as part of the Continuous Integration, in [`.github/workflows`](.github/workflows).

## Contributing

Refer to the [Contributing page](CONTRIBUTING.md).

## Maintainers

- Silvio Traversaro ([@traversaro](https://github.com/traversaro))
- Alessandro Croci ([@xela-95](https://github.com/xela-95))
