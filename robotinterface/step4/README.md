## robotinterface - step 4

#### Problem
Implement a simple Gazebo plugin that loads the .xml file via the `libYARP_robotinterface` library launches the `multipleanalogsensorsserver` device
  
#### Run the solution

At first, compile launch.cc:
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
  yarp read /read /prova/measures:o
  ~~~

The output is:
~~~
(((229.0 229.0 229.0) 1686813460.49840688705)) (((-7.39064523333979739306 0.0 6.45092730038959061289) 1686813460.49840688705)) (((229.0 229.0 229.0) 1686813460.49840688705)) (((229.0 229.0 229.0) 1686813460.49840688705)) () () () () () ()
(((239.0 239.0 239.0) 1686813460.60061335564)) (((-8.39810790494276204754 0.0 5.07029423376374932531) 1686813460.60061335564)) (((239.0 239.0 239.0) 1686813460.60061335564)) (((239.0 239.0 239.0) 1686813460.60061335564)) () () () () () ()
...
~~~

