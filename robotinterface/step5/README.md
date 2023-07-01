## robotinterface - step 5

#### Problem
* Implement a YARP device that inherits from [`yarp::dev::ISixAxisForceTorqueSensors`](https://yarp.it/latest/classyarp_1_1dev_1_1ISixAxisForceTorqueSensors.html), that reads the values from the gazebo simulation
* Publish the data from the YARP device to a YARP port by launching a multipleanalogsensorsserver via the yarprobotinterface C++ api
  
#### Run the solution

At first, compile:
~~~
mkdir build
cd build
cmake ..
make
~~~

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
  export LIBGL_ALWAYS_SOFTWARE=1 
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
() () () () () (((0.0 0.0 98.0 0.0 0.0 0.0) 0.0060000000000000001249)) () () () ()
() () () () () (((0.0 0.0 98.0 0.0 0.0 0.0) 0.087999999999999994893)) () () () ()
() () () () () (((0.0 0.0 98.0 0.0 0.0 0.0) 0.152999999999999997113)) () () () ()
() () () () () (((0.0 0.0 98.0 0.0 0.0 0.0) 0.234000000000000013545)) () () () ()
...
~~~

