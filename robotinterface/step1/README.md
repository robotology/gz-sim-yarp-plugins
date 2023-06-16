## robotinterface - step 1

#### Problem
To get use to the use of yarprobotinterface files, write a .xml file that launches two devices, and attach them.
   * `fakeIMU` : a device that exposes a fake IMU that measures orientation, angular velocity and linear acceleration
   * [`multipleanalogsensorsserver`](https://www.yarp.it/latest/classMultipleAnalogSensorsServer.html) : a device that read the measures from the `fakeIMU`, and publish them on the YARP network
   * You can test this file by launching the `yarprobotinterface` executable
  
#### Run the solution
- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  yarprobotinterface --config prova.xml
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

