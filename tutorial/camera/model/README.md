# Run Camera Plugin in Gazebo Garden

## Run model in Gazebo Garden with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  export LIBGL_ALWAYS_SOFTWARE=1
  cd build
  export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`:`pwd`/lib
  cd ../tutorial/camera/model
  gz sim model.sdf
  ~~~
- 3rd terminal:
  ~~~
  yarpview
  ~~~
- 4th terminal:
  ~~~
  yarp connect /camera /yarpview/img:i
  ~~~

Finally start the simulation in Gazebo.
- yarpview window
  ![yarpview window](imgs/yarpview.png "yarpview window")
- Gazebo simulation
  ![Gazebo simulation](imgs/simulation.png "Gazebo simulation")



