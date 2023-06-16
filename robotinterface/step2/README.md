## robotinterface - step 2

#### Problem
Once you tested successfully the .xml file with the `yarprobotinterface` (see [step1](https://github.com/robotology/study-gazebo-garden-yarp-plugins/edit/main/robotinterface/step1)), write a simple C++ program that uses the `libYARP_robotinterface` library ( https://www.yarp.it/latest/group__robointerface__all.html ) to load the .xml file and launch it, without passing through the `yarprobotinterface` executable (but you can check the `yarprobotinterface` executable code to get an inspirtation)
  
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
  ./build/launch
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

