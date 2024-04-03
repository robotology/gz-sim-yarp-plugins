# gz-sim-yarp-plugins

> [!WARNING]  
> This repository contains a preliminary work in progress of integration of Modern Gazebo (gz) and YARP devices, a port of some functionalities of <https://github.com/robotology/gazebo-yarp-plugins> to Modern Gazebo. \
> The repo is working in progress, and public interfaces can change without warning.

## Installation

At the moment we do not provide any binary for `gz-sim-yarp-plugins`, so you need to compile it from source, either
installing the dependencies with conda-forge on Linux, macOS or Windows, or apt on Ubuntu.

### Compile from source using conda-forge dependencies on Linux, macOS or Windows

Create and activate an environment with the required dependencies:

~~~
mamba create -c conda-forge -n gsypdev libgz-sim8 yarp ycm-cmake-modules cmake ninja pkg-config cmake compilers gtest
mamba activate gsypdev
~~~

All the commands in this README should be executed in a terminal with the activated environment.

Then, compile gz-sim-yarp-plugins itself, using the following commands on Linux and macOS:

~~~
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX ..
ninja
ninja install
~~~

or the following commands on Windows:

~~~
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=%CONDA_PREFIX%\Library -DCMAKE_PREFIX_PATH=%CONDA_PREFIX%\Library ..
ninja
ninja install
~~~

### Compile from source using apt dependencies on Linux, macOS or Windows

First install some necessary dependencies from apt  

~~~
sudo apt-get update
sudo apt-get install lsb-release wget gnupg cmake pkg-config ninja-build build-essential libgtest-dev
~~~

Then install Gazebo Harmonic:

~~~
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
~~~

Then, you need to install [`ycm-cmake-modules`](https://github.com/robotology/ycm) and [`yarp`](https://github.com/robotology/yarp), for which no apt binaries are available. You can install them easily via the `robotology-superbuild`, or otherwise with the following commands:

~~~
mkdir ~/gsyp_ws
cd ~/gsyp_ws
git clone https://github.com/robotology/ycm
git clone https://github.com/robotology/yarp
cd ycm
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=~/gsyp_ws/install -DCMAKE_PREFIX_PATH=~/gsyp_ws/install ..
ninja
ninja install
cd ~/gsyp_ws/yarp
cd yarp
git checkout v3.9.0
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=~/gsyp_ws/install -DCMAKE_PREFIX_PATH=~/gsyp_ws/install ..
ninja
ninja install
~~~

Then, install `gz-sim-yarp-plugins` itself:

~~~
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=~/gsyp_ws/install -DCMAKE_PREFIX_PATH=~/gsyp_ws/install ..
ninja
ninja install
~~~

## Usage

To notify Gazebo of the new plugins compiled, it is necessary to modify the `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable, for example on Linux:

~~~
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:<install_location>/lib
~~~

where `<install_location>` is the directory passed to `CMAKE_INSTALL_PREFIX` during the CMake configuration.

Once the plugins are available, you can see how to use the different plugins by looking in the directories contained in the `tutorial` folder of this repo. Each directory is an example, and contains a README that shows how to run that example.

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
