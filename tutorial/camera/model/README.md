# Run Camera Plugin in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  - Update the `GZ_SIM_RESOURCE_PATH` environment variable to point to the `tutorial` folder:

    ~~~
    export GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH:<path-to-tutorial-folder>
    ~~~

  - Then, launch Gazebo:

    ~~~
    cd <path-to-tutorial-folder>/camera/model
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

If you are using Linux on WSLg and you have the following error
~~~
OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo at ./RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 685)
~~~
try forcing software rendering:
~~~
export LIBGL_ALWAYS_SOFTWARE=1
~~~


