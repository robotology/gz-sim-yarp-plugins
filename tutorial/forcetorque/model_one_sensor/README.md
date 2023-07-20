# Run forcetorque in Gazebo Garden

## Run model in Gazebo Garden with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  export LIBGL_ALWAYS_SOFTWARE=1 
  cd ../../../build
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`
  cd ../tutorial/forcetorque/model_one_sensor
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

## Run model in Gazebo Garden without YARP integration
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
