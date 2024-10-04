# Build gz-sim-yarp-plugins from source

## Table of Contents

- [Dependencies](#dependencies)
- [Compile from source with pixi on Linux, macOS or Windows](#compile-from-source-with-pixi-on-linux-macos-or-windows)
- [Compile from source with conda on Linux, macOS or Windows](#compile-from-source-with-conda-on-linux-macos-or-windows)
- [Compile from source using apt dependencies on Ubuntu Linux](#compile-from-source-using-apt-dependencies-on-ubuntu-linux)
- [Usage](#usage)

## Dependencies

`gz-sim-yarp-plugins` is a fairly classical C++ project build with CMake, so it should be quite easy to build if you are already familiar with how you build C++ projects with CMake.
If you are not familiar with the use of CMake, you can check some documentation on <https://cmake.org/runningcmake/> or <https://cgold.readthedocs.io>.

Before building `gz-sim-yarp-plugins`, you need to install its dependencies, the main ones being:

- [Modern Gazebo](https://gazebosim.org/home)
- [YARP](https://yarp.it/latest//)
- [YCM CMake Modules](https://robotology.github.io/ycm/gh-pages/latest/index.html#)

in addition to the usual dependencies used to configure, compile and test C++ packages.

To compile gz-sim-yarp-plugins from source, follow just one out of the following sections.

## Compile from source with pixi on Linux, macOS or Windows

If you want to use [pixi](https://pixi.sh) to compile the project, just clone the repo and in the `gz-sim-yarp-plugins` folder run:

~~~
pixi run build
~~~

This will automatically download all dependencies, and build the project.

To run the test suite, run:

~~~
pixi run test
~~~

To install the project in the environment created by pixi, run:
~~~
pixi run install
~~~

Then you can launch programs in this environment with `pixi run`, for example:

~~~
pixi run gz sim
~~~

If you prefer to launch programs with prepending them with `pixi run`, you can activate the pixi environment with:

~~~
pixi shell
~~~

and then run your commands as in a normal shell.

The `default` environment of pixi uses Gazebo Harmonic (`gz-sim8`), if you want to use Gazebo Ionic (`gz-sim9`) just add `-e ionic` after `pixi run`, for example to run the tests under Ionic:

~~~
pixi run -e ionic test
~~~

## Compile from source with conda on Linux, macOS or Windows

If you are using conda, the dependencies of `gz-sim-yarp-plugins` can be installed with:

```bash
conda install -c conda-forge libgz-sim8 yarp ycm-cmake-modules cmake ninja pkg-config cmake compilers gtest cli11
```

This command should be executed in a terminal with the environment activated. If you want to use Gazebo Ionic (`gz-sim9`) in place of Gazebo Harmonic (`gz-sim8`), just change `libgz-sim8` to `libgz-sim9`.

### Build

You can then compile abd install `gz-sim-yarp-plugins` itself, using the following commands on **Linux** and **macOS**:

```bash
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DCMAKE_PREFIX_PATH=$CONDA_PREFIX ..
ninja
ninja install
```

or the following commands on **Windows**:

```bash
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=%CONDA_PREFIX%\Library -DCMAKE_PREFIX_PATH=%CONDA_PREFIX%\Library ..
ninja
ninja install
```

## Compile from source using apt dependencies on Ubuntu Linux

If you are using an apt-based distribution such as Ubuntu and you want to use apt, the dependencies can be installed via:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg cmake pkg-config ninja-build build-essential libcli11-dev libgtest-dev
```

and then install Gazebo:

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

If you want to use Gazebo Ionic (`gz-sim9`) in place of Gazebo Harmonic (`gz-sim8`), just change `gz-harmonic` to `gz-ionic`.

Then, you need to install [`ycm-cmake-modules`](https://github.com/robotology/ycm) and [`yarp`](https://github.com/robotology/yarp), for which no apt binaries are available. You can install them easily via the `robotology-superbuild`, or otherwise with the following commands:

```bash
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
```

Then, build and install `gz-sim-yarp-plugins` itself:

```bash
git clone https://github.dev/robotology/gz-sim-yarp-plugins
mkdir build
cd build
cmake -GNinja -DCMAKE_INSTALL_PREFIX=~/gsyp_ws/install -DCMAKE_PREFIX_PATH=~/gsyp_ws/install ..
ninja
ninja install
```

## Usage

To notify Gazebo of the new plugins compiled, unless you are using pixi it is necessary to modify the `GZ_SIM_SYSTEM_PLUGIN_PATH` environment variable, for example on Linux:

~~~
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:<install_location>/lib
~~~

where `<install_location>` is the directory passed to `CMAKE_INSTALL_PREFIX` during the CMake configuration.

To ensure that the tools are found, instead you need to add their location to the `PATH`:

~~~
export PATH=${PATH}:<install_location>/bin
~~~

