### gzyarp::ForceTorque Plugin


| `name`          | `filename`                    |
|:---------------:|:-----------------------------:|
| `gzyarp::ForceTorque` | `gz-sim-yarp-forcetorque-system` |

The `gzyarp::ForceTorque` plugin exposes the measurements of a `ForceTorque` sensor attached to a joint in a `gz-sim` model as a YARP devices that exposes the `yarp::dev::ISixAxisForceTorqueSensors`. This allows you to access simulated 6-axis force/torque data (three forces and three torques) through standard YARP interfaces and tools.

### Usage

Add the `gzyarp::ForceTorque` plugin to the model SDF where the force–torque sensor is defined. You must specify a YARP configuration (either as a string or via file, see the main repository `README.md` section “How to specify Yarp configurations”), and at minimum provide:

- `sensorName`: the name of the `ForceTorque` sensor attached to the joint.
- `jointName`: the name of the joint on which the sensor is mounted.
- `yarpDeviceName`: the instance name of the YARP device that will be created and registered in the `DeviceRegistry`.

A concrete usage can be seen in the [`tutorial/forcetorque/model_one_sensor`](../../tutorial/forcetorque/model_one_sensor) folder. The relevant part of the `model.sdf` is:

```xml
        <model name="force_torque_model">
            <plugin name="gzyarp::ForceTorque" filename="gz-sim-yarp-forcetorque-system">
                <yarpConfigurationString>
                    (yarpDeviceName forcetorque_plugin_device)
                    (jointName joint_12)
                    (sensorName force_torque_sensor)
                </yarpConfigurationString>
            </plugin>
            ...
            <joint name="joint_12" type="revolute">
                <parent>link_1</parent>
                <child>link_2</child>
                <sensor name="force_torque_sensor" type="force_torque">
                    <always_on>true</always_on>
                    <visualize>true</visualize>
                    <update_rate>30</update_rate>
                </sensor>
            </joint>
            <plugin name="gzyarp::RobotInterface" filename="gz-sim-yarp-robotinterface-system">
                <yarpRobotInterfaceConfigurationFile>
                    model://forcetorque/model_one_sensor/forcetorque_nws.xml
                </yarpRobotInterfaceConfigurationFile>
            </plugin>
        </model>
```

The `forcetorque_nws.xml` robotinterface file that exposes the measurements on a YARP port is:

```xml
<robot name="forcetorque" portprefix="forcetorque">
    <devices>
        <device name="forcetorque_nws_yarp" type="multipleanalogsensorsserver">
            <param name="name"> /forcetorque </param>
            <param name="period"> 100 </param>
            <action phase="startup" level="5" type="attach">
                <param name="device"> forcetorque_plugin_device </param>
            </action>
            <action phase="shutdown" level="5" type="detach" />
        </device>
    </devices>
</robot>
```

Once the plugin is loaded correctly, a YARP device with instance name `forcetorque_plugin_device` is available in the `DeviceRegistry`. The `gzyarp::RobotInterface` plugin then attaches a `multipleanalogsensorsserver` to it, which exposes the measurements on `/forcetorque/measures:o`.

### Reference documentation of `plugin` child XML elements

The `gzyarp::ForceTorque` plugin itself does not directly parse custom child XML elements beyond the generic YARP configuration helpers. All configuration is provided through the standard YARP configuration mechanisms of `yarpConfigurationFile` or `yarpConfigurationString`.

The following YARP parameters are **required** inside the configuration (regardless of using `yarpConfigurationFile` or `yarpConfigurationString`):

| YARP parameter   | Description |
|:----------------:|:------------|
| `sensorName`     | Name of the `ForceTorque` sensor in the model to be read. |
| `jointName`      | Name of the joint the sensor is attached to, used to resolve the sensor entity. |
| `yarpDeviceName` | Instance name of the YARP device, used to register it in the `DeviceRegistry` and to access it from other plugins (for example through `gzyarp::RobotInterface` or `gzyarp::ConfigurationOverride`). |

For more details on how to run the example and how the YARP ports look like, see the READMEs in [`tutorial/forcetorque/model_one_sensor`](../../tutorial/forcetorque/model_one_sensor) and [`tutorial/forcetorque/model_two_sensors`](../../tutorial/forcetorque/model_two_sensors), and the general information on YARP configurations in the main repository [`README.md`](../../README.md).
