# Run clock tutorial in Gazebo Garden

The clock plugin sends on the `/clock` port the current simulation time in **nanoseconds**.

## Run model in Gazebo Garden with YARP integration

- 1st terminal:

  ~~~
  yarp server
  ~~~

- 2nd terminal:

  ~~~
  cd tutorial/clock
  gz sim model.sdf
  ~~~

- 3rd terminal:

  ~~~
  yarp name list
  yarp read /read ... /clock
  ~~~

Finally start the simulation in Gazebo. You will read the simulation time increasing, as in the following:

~~~
$ yarp read "..." /clock
[INFO] |yarp.os.Port|/tmp/port/2| Port /tmp/port/2 active at tcp://192.168.158.201:10004/
[INFO] |yarp.os.impl.PortCoreInputUnit|/tmp/port/2| Receiving input from /clock to /tmp/port/2 using tcp
1000000
2000000
3000000
4000000
5000000
6000000
7000000
8000000
9000000
10000000
11000000
12000000
13000000
14000000
15000000
16000000
17000000
18000000
19000000
~~~

> [!NOTE]  
> When simulation is **paused**, simulation time does not increase and the clock plugin will not send timestamps on the `/clock` port until simulation is resumed.

> [!CAUTION]  
> When simulation is **reset**, simulation time will restart from zero. Clock plugin manages this event without closing the `/clock` port. Please pay attention to potential unexpected behaviors of subscribers when a reset occurs.
