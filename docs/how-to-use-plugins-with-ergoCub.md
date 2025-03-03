# How to use `gz‐sim‐yarp‐plugins` with ergoCub

This document explains how to install and use ergoCub (or the old iCub 2.*) in the Modern Gazebo (gz-sim) simulator through `gz-sim-yarp-plugins`.

> [!NOTE]
> For the time being this guide will provide operative instructions only for **Linux** distributions; instructions will also be provided for Windows and macOS in the future.

Note that some features of the ergocub model (mainly the depth sensors and the hands) are not supported, see <https://github.com/robotology/gz-sim-yarp-plugins/issues/137> for more details.

## Table of Contents

- [Installation](#installation)
- [World creation](#world-creation)
- [Run the simulation](#run-the-simulation)

## Installation

The easiest ways to setup an environment with everything we need is to follow only one of the following two options:

- [Creating a conda environment](#conda-environment)
- [Using the robotology-superbuild](#installation-via-robotology-superbuild)


### Conda environment

#### Install a conda distribution

If you do not have one, please install a conda distribution. We suggest to use the minimal [miniforge](https://github.com/conda-forge/miniforge) distribution, which uses conda-forge packages by default.

To install miniforge, please follow the instructions in [install-miniforge](https://github.com/robotology/robotology-superbuild/blob/master/doc/install-miniforge.md) documentation.

#### Create an environment

Then create the following environment with all the packages needed:

~~~bash
conda create -n ergocub-gz libgz-sim-yarp-plugins ergocub-software
~~~

Once you did this, you can activate it with:

~~~bash
conda activate ergocub-gz
~~~

### Installation via robotology-superbuild

[robotology-superbuild](https://github.com/robotology/robotology-superbuild) is a meta repository that is able to download and compile a plethora of software from the [robotology](https://github.com/robotology) organization.

First of all, we have to configure the superbuild such that it will download and install the software we need. It is possible to do this by enabling the appropriate options: in particular, we need to enable the `ROBOTOLOGY_ENABLE_CORE` profile with `ROBOTOLOGY_USES_GZ` option and disable the `ROBOTOLOGY_USES_GAZEBO` one.

Follow the [source installation guide](https://github.com/robotology/robotology-superbuild/tree/master?tab=readme-ov-file#source-installation) to install the required software.

## World creation

In Modern Gazebo the preferred way to initialize the simulator is by providing a **World**: an SDF file that contains everything we need to simulate our scenario.

In a directory of your choice, create a file called `ergocub-world.sdf` and fill it by copy-pasting the following content:

```xml
<?xml version="1.0"?>

<sdf version="1.11">
    <world name="tutorial_controlboard">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact">
        </plugin>

        <!-- YARP CLOCK -->
        <plugin
            filename="gz-sim-yarp-clock-system"
            name="gzyarp::Clock">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.8 1</ambient>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                </visual>
            </link>
        </model>

        <include>
            <uri>model://ergoCub/robots/ergoCubGazeboV1_1</uri>
        </include>

    </world>
</sdf>
```

To use a iCub 2.* model in place of ergoCub, use `<uri>package://iCub/robots/iCubGazeboV2_7/model.urdf</uri>` or `<uri>package://iCub/robots/iCubGazeboV2_5/model.urdf</uri>` (or any model you want as documented in https://github.com/robotology/icub-models?tab=readme-ov-file#model-details) in the world.

## Run the simulation

To run the simulation, open a terminal, navigate to the folder containing the world file (otherwise prefix the world filename with the path to it) and execute the following command:

```bash
gz sim ergocub-world.sdf
```

The simulator should open in a paused state with ergoCub in place. To start the simulation just click on the play button on the bottom left.

![image](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/50b24661-5885-4314-be6e-2827d45e6908)

### Gazebo + YARP = ❤️

Things get interesting when `gz-sim-yarp-plugins` are used to connect the Gazebo simulation via to YARP devices able to interact with it.

For example, one could use [yarpmotorgui](https://yarp.it/latest//group__yarpmotorgui.html) to control the robot joints.

To do this, we need three terminals:

1. In the first one, start a `yarp server` by executing:

```bash
yarp server
```

2. In the second terminal, launch the simulator by executing:

```bash
gz sim ergocub-world.sdf
```

and start the simulation;
3. In the third one launch the `yarpmotorgui` by executing (if you are using iCub 2.*, pass `--robot icubSim`):

```bash
yarpmotorgui --robot ergocubSim
```

Then click Ok on the panel asking which robot parts to control and finally you should be able to control each joint of the ergoCub robot:
![image](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/9247b83f-9333-4e44-bc5b-f65cbe592d89)
