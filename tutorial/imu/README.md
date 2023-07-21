# Run IMU Plugin in Gazebo Garden

## Run model in Gazebo Garden with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  export LIBGL_ALWAYS_SOFTWARE=1 
  cd ../../build
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`
  cd ../tutorial/imu
  gz sim model.sdf
  ~~~
- 3rd terminal:
  ~~~
  yarp name list
  yarp read /read /IMU/measures:o
  ~~~

Finally start the simulation in Gazebo. The output is:
~~~
(((0.0 0.0 0.0) 0.790000000000000035527)) (((0.0 0.0 0.0) 0.790000000000000035527)) () (((0.0 0.0 1.0) 0.790000000000000035527)) () () () () () ()

(((-8.67083959466306219856e-18 -9.04614162502292736348e-17 6.31690337169893099938e-18) 0.856999999999999984013)) (((-3.75849095399637712398e-15 -5.98854508443207591236e-16 9.79999999999999715783) 0.856999999999999984013)) () (((-4.6463335023590564651e-18 7.44705833710973762087e-18 1.0) 0.856999999999999984013)) () () () () () ()

(((-2.65163859086565008197e-17 -6.92733521733797742996e-17 1.07010834988178922973e-17) 0.918000000000000038192)) (((-4.09782279403881113319e-15 5.95134124642898954114e-16 9.79999999999999715783) 0.918000000000000038192)) () (((-4.95593801967022345119e-18 7.07755278231224452645e-18 1.0) 0.918000000000000038192)) () () () () () ()
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
Print the topic that correspond to the IMU sensor:
```
gz topic -e -t <imu_topic_name>
```
where you should substitute `<imu_topic_name>` with the topic of the IMU name.    
Youl should see an output like this:
```
header {
  stamp {
    nsec: 950000000
  }
  data {
    key: "frame_id"
    value: "sensor_box::link_1::imu_sensor"
  }
  data {
    key: "seq"
    value: "95"
  }
}
entity_name: "sensor_box::link_1::imu_sensor"
orientation {
  x: -5.1247474484916385e-18
  y: 6.78498225005388e-18
  w: 1
}
angular_velocity {
  x: -1.4944019409287915e-18
  y: 9.4674654834166837e-18
  z: 4.3001709350815993e-19
}
linear_acceleration {
  x: -7.07328548583498e-16
  y: 2.4369443497133971e-15
  z: 9.8
}
...
```


