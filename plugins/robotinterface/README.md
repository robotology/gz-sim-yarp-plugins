### gzyarp::RobotInterface Plugin


| `name`        | `filename`         |
|:-------------:|:------------------:|
| `gzyarp::RobotInterface` |  `gz-sim-yarp-robotinterface-system` |

The `gzyarp::RobotInterface` plugin is used to instantiate arbitrary YARP devices as part of a `gz-sim` simulation, using the [YARP RobotInterface library](https://www.yarp.it/latest/group__robointerface__all.html).

Particularly, tipically the YARP devices that simulate hardware devices are instantiated via dedicated `gz-sim-yarp-*` `gz-sim` plugins, while to launch any other YARP device, for example [Network Wrapper Servers](https://yarp.it/latest/group__nws__and__nwc__architecture.html) to expose YARP devices functionalities in dedicated middlewares like YARP ports or ROS2 topics, you need to use the `gzyarp::RobotInterface` plugin.

### Usage

Add the `gzyarp::RobotInterface` plugin to the model SDF after all the plugin declarations that instantiated YARP devices that you want to use.

An example of its usage can be seen in the [`tutorial/single_pendulum/single_pendulum_world.sdf`](../../tutorial/single_pendulum/model.sdf)

A more complete example of the usage is:

```xml
        <plugin name="gzyarp::ControlBoard" filename="gz-sim-yarp-controlboard-system">
            <yarpConfigurationFile>
                model://single_pendulum/conf/gazebo_controlboard.ini
            </yarpConfigurationFile>
        </plugin>
        <!-- Launch the other YARP devices, in this case a controlBoard_nws_yarp to expose the
        controlboard functionalities via YARP ports -->
        <plugin name="gzyarp::RobotInterface" filename="gz-sim-yarp-robotinterface-system">
            <yarpRobotInterfaceConfigurationFile>
                model://single_pendulum/conf/single_pendulum_nws.xml
            </yarpRobotInterfaceConfigurationFile>
            <!-- This element can be used to pass "enable" tags to the libYARP_robotinterface instance -->
            <yarpRobotInterfaceEnableTags>
            (enable_tag1 enable_tag2)
            </yarpRobotInterfaceEnableTags>
            <!-- This element can be used to pass "disable" tags to the libYARP_robotinterface instance -->
            <yarpRobotInterfaceDisableTags>
            (disable_tagA)
            </yarpRobotInterfaceDisableTags>
            <!-- This element can be used to override the port prefix with the top-level model name (default: false) -->
            <yarpRobotInterfaceOverridePortPrefixWithModelName>true</yarpRobotInterfaceOverridePortPrefixWithModelName>
        </plugin>
```

The `yarpRobotInterfaceEnableTags` and `yarpRobotInterfaceDisableTags` can also be overriden via the `gzyarp::ConfigurationOverride` plugin,
for example if `model://model_with_robotinterface_inside` is a gz-sim model with a `gzyarp::RobotInterface` plugin inside, you can specify the `enable_tags` or `disable_tags` to specify with:

```xml
    <model name="robotinterface_with_overriden_tags">
        <plugin name='gzyarp::ConfigurationOverride' filename='gz-sim-yarp-configurationoverride-system'>
            <!-- The yarpPluginConfigurationOverride device is used to override the configuration of
                 a given gzyarp::RobotInterface plugin, whose name (the one specified inside the xml file) is specified with yarpRobotInterfaceName, or you can specify `all` to ensure that the enable tags will be specified for all -->
            <yarpPluginConfigurationOverride yarpRobotInterfaceName='all'/>
            <yarpRobotInterfaceEnableTags>(enable_tag1)</yarpRobotInterfaceEnableTags>
            <yarpRobotInterfaceDisableTags>(disable_tagA disable_tagB)</yarpRobotInterfaceDisableTags>
            <yarpRobotInterfaceOverridePortPrefixWithModelName>true</yarpRobotInterfaceOverridePortPrefixWithModelName>
        </plugin>
        <include>
            <uri>model://model_with_robotinterface_inside</uri>
        </include>
    </model>
```

### Reference documentation of `plugin` child XML elements

| XML element | Documentation |
|:------------:|:-----------------------:|
| `yarpRobotInterfaceConfigurationFile` | The [YARP robotinterface XML file](https://www.yarp.it/latest/group__yarp__robotinterface__xml__config__files.html) that specifies the YARP devices that will be launched by the `gzyarp::RobotInterface` plugin. |
| `yarpRobotInterfaceEnableTags` | This element can be used to pass "enable" tags to the libYARP_robotinterface instance. |
| `yarpRobotInterfaceDisableTags` | This element can be used to pass "disable" tags to the libYARP_robotinterface instance. |
| `yarpRobotInterfaceOverridePortPrefixWithModelName` | Boolean flag (default: `false`). When set to `true`, the port prefix specified in the robotinterface XML file will be overridden with the top-level model name (the model name specified at the world level in the SDF). When `false`, the original port prefix from the XML file is preserved. |