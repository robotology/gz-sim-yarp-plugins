# Run Camera Plugin in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  - Update the `GZ_SIM_RESOURCE_PATH` environment variable to point to the `tutorial` folder:

    ~~~
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path-to-tutorial-folder>
    ~~~

  - Then, launch Gazebo:

    ~~~
    cd <path-to-tutorial-folder>/camera/model_horizontal_flip
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
  ![yarpview window](imgs/yarpview_horizontal_flip.png "yarpview window")
- Gazebo simulation
  ![Gazebo simulation](imgs/simulation.png "Gazebo simulation")
