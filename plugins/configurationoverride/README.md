### gzyarp::ConfigurationOverride Plugin


| `name`        | `filename`         |
|:-------------:|:------------------:|
| `gzyarp::ConfigurationOverride` |  `gz-sim-yarp-configurationoverride-system` |

The `gzyarp::ConfigurationOverride` plugin can be attached to a model in order to override the configuration of any other `gz-sim-yarp` plugin that is attached to the same model
and that occurs in `xml` after the insertion of the ConfigurationOverride plugin, even in a nested model.

Each ConfigurationOverride plugin points to another plugin using a unique identifier, and can override the values of its configuration parameters.


### Usage

Add the `gzyarp::ConfigurationOverride` plugin to the model SDF before the overriden plugin declaration (i.e. before the `<include>`) and define the `yarpDeviceName` attribute of the `yarpPluginConfigurationOverride` element as the `yarpDeviceName` of the YARP plugin that will be overriden.

A typical usage of the `gzyarp::ConfigurationOverride` plugin is to specify parameters of a model included in a world, without the need to
modify the model itself, that is convenient when the model is installed by another package and so it cannot be easily modified.

An example of its usage can be seen in the [`tutorial/single_pendulum/single_pendulum_world.sdf`](../../tutorial/single_pendulum/single_pendulum_world.sdf)

The relevant snippet is:

```xml
    <!-- This is an example of how to override the initial configuration -->
    <plugin name='gzyarp::ConfigurationOverride' filename='gz-sim-yarp-configurationoverride-system'>
      <!-- The yarpPluginConfigurationOverride device is used to override the configuration of
           a given yarp device, whose name is specified via the yarpDeviceName attribute -->
      <yarpPluginConfigurationOverride yarpDeviceName='controlboard_plugin_device'/>

      <!-- If you want to override the initialConfiguration xml element, set it -->
      <initialConfiguration>0.785398</initialConfiguration>
    </plugin>
```

The original `initialConfiguration` of the `controlboard_plugin_device` was `0.0`, while via the `gzyarp::ConfigurationOverride` plugin we override its value to be `0.785398` rad (i.e. 45 degrees).

Note that the `<plugin name='gzyarp::ConfigurationOverride' [...]>` element is child of another parent `<model>`.

This is required as Gazebo Harmonic (`gz-sim8`) there are some inconsistency related to the scoped name of the entity passed to plugin inserted under the `world` element. If you are using Gazebo Ionic (`gz-sim9`) or later, you can also insert directly the  `<plugin name='gzyarp::ConfigurationOverride' [...]>` as a child of the `<world>` element.

Another tipical example of `gzyarp::ConfigurationOverride` plugin is to set the enable and disable tags of a `gzyarp::RobotInteface` plugin, you can read more about that in [`gzyarp::RobotInterface` documentation](../robotinterface/README.md).

### Reference documentation of `plugin` child XML elements

| XML element | Documentation |
|:------------:|:-----------------------:|
| `yarpPluginConfigurationOverride` | The attributes of this element specify the plugin of which we will override the parameters. Possible attributes are `yarpDeviceName`, that specifies the `yarpDeviceName='@yarpDeviceName@'` instantiated by the plugin to override, or `yarpRobotInterfaceName='@yarpRobotInterfaceName@'` to override the parameters of  `gzyarp::RobotInterface` plugins named `yarpRobotInterfaceName`. In both cases, you can pass `all` to override the parameters of all YARP devices or YARP RobotInterface plugins in all nested models. |

The parameters and plugins currently supported by the `gzyarp::ConfigurationOverride` plugin are the following:

| Parameter SDF Element name       | Overrided Plugin |
|:--------------------------------:|:---------------------:|
| `initialConfiguration`           | `gzyarp::ControlBoard` |
| `yarpRobotInterfaceEnableTags`   | `gzyarp::RobotInterface` |
| `yarpRobotInterfaceDisableTags`  | `gzyarp::RobotInterface` |


Any other parameter is not overriden. The plan for the future is to support more parameters in the `gzyarp::ConfigurationOverride`, if there is some specific parameter that you would like to override via the `gzyarp::ConfigurationOverride`, please [open an issue](https://github.com/robotology/gz-sim-yarp-plugins/issues/new).
