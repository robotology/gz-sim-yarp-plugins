# Run forcetorque in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  - Update the `GZ_SIM_RESOURCE_PATH` environment variable to point to the `tutorial` folder:

    ~~~
    export GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH:<path-to-tutorial-folder>
    ~~~

  - Then, launch Gazebo:

    ~~~
    cd <path-to-tutorial-folder>/forcetorque/model_one_sensor
    gz sim model.sdf
    ~~~

- 3rd terminal:
  ~~~
  yarp name list
  yarp read /read /forcetorque/measures:o
  ~~~

Finally start the simulation in Gazebo. The output is:
~~~
() () () () () (((0.0 0.0 0.0 0.0 0.0 0.0) 0.0)) () () () ()
() () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 0.030999999999999999778)) () () () ()
() () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 0.0749999999999999972244)) () () () ()
() () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 0.140999999999999986455)) () () () ()
...
~~~

If you are using Linux on WSLg and you have the following error
~~~
OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo at ./RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 685)
~~~
try forcing software rendering:
~~~
export LIBGL_ALWAYS_SOFTWARE=1
~~~

## Run model in Gazebo without YARP integration
Run the model:
```
gz sim model.sdf
```
List all available Gazebo topics:
```
gz topic -l
```
Print the topic that correspond to the FT sensor:
```
gz topic -e -t <ft_topic_name>
```
where you should substitute `<ft_topic_name>` with the topic of the FT name.
Youl should see an output like this:
```
header {
  stamp {
    nsec: 1000000
  }
  data {
    key: "frame_id"
    value: "force_torque_example::joint_12::force_torque"
  }
  data {
    key: "seq"
    value: "0"
  }
}
force {
  z: -98
}
torque {
}
...
```
We have 3 values for force and 3 values for torque (if the value is 0, it is not present).
In this case, it is possible to see that the force measure make sense as the weight of the link is 10 Kg, and the acceleration of gravity is 9.8, so the measure norm on the Z axis is correctly -9.8*10 = -98.

If you try to interact with the model, for example in Gazebo GUI by dropping a cylinder on top of it, you can see how the values of the FT sensor change:
```
header {
  stamp {
    sec: 2
    nsec: 211000000
  }
  data {
    key: "frame_id"
    value: "force_torque_example::joint_12::force_torque"
  }
  data {
    key: "seq"
    value: "67"
  }
}
force {
  x: -2.2791356195958095
  y: 0.095224507948984877
  z: -106.04082176280804
}
torque {
  x: 0.39727986274295268
  y: 0.28808430716061151
  z: 1.3552527156068805e-17
}
...
```
