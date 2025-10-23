# Run single pendulum tutorial in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:

  ~~~bash
  yarp server
  ~~~

- 2nd terminal:
  - Update the `GZ_SIM_RESOURCE_PATH` environment variable to point to the `tutorial` folder:

    ~~~
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path-to-tutorial-folder>
    ~~~

  - Then, launch Gazebo:

    ~~~
    cd <path-to-tutorial-folder>/single_pendulum
    gz sim single_pendulum_world.sdf
    ~~~

  The Gazebo GUI will open and the single pendulum should be already spawned in the scene.

  ![gz_gui](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/508f767c-9a8c-460c-87c7-9964fa01261b)

- 3rd terminal:

  ~~~bash
    cd tutorial/single_pendulum
    yarpmotorgui
  ~~~

  The [yarpmotorgui](https://www.yarp.it/latest/group__yarpmotorgui.html) interface will open and it should automatically prompt you to select the `/singlePendulumGazebo/body` part.

  Click Ok.

  ![yarpmotorgui_select_part](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/5f962770-b08e-4f30-9990-5b2940da5811)

Finally start the simulation in Gazebo (click on the Play button on the bottom left). The pendulum will start to swing freely since the default control mode is **Idle**.

Click on Idle and select **Torque**. A slider will appear on which it is possible to manually set the reference torque [Nm].

![yarpmotorgui_torque](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/53f87d7f-9cad-4e24-9c43-5d227368aa3f)

Here's an example of the final result:
[tutorial_video](https://github.com/robotology/gz-sim-yarp-plugins/assets/57228872/396992b5-f627-447e-b6b9-e3a122819ef7)
