# Run forcetorque in Gazebo

## Run model with two FT sensors in Gazebo

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
    cd <path-to-tutorial-folder>/forcetorque/model_two_sensors
    gz sim model2sensors.sdf
    ~~~

- 3rd terminal:
    The output of the command `yarp name list` is:
    ~~~
    registration name /forcetorque1/measures:o ip 172.27.5.40 port 10002 type tcp
    registration name /forcetorque1/rpc:o ip 172.27.5.40 port 10003 type tcp
    registration name /forcetorque2/measures:o ip 172.27.5.40 port 10004 type tcp
    registration name /forcetorque2/rpc:o ip 172.27.5.40 port 10005 type tcp
    registration name /root ip 172.27.5.40 port 10000 type tcp
    registration name fallback ip 224.2.1.1 port 10000 type mcast
    ~~~

    We can see the measures of the two sensors with the command:
  - `yarp read /readft1 /forcetorque1/measures:o`

    ~~~
    () () () () () (((0.0 0.0 0.0 0.0 0.0 0.0) 0.0)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.0389999999999999999445)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.115000000000000004996)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.193000000000000004885)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.275000000000000022204)) () () () ()
    ...
    ~~~

  - `yarp read /readft2 /forcetorque2/measures:o`
    ~~~
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.53100000000000058265)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.61299999999999954525)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.68900000000000005684)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.77299999999999968736)) () () () ()
    ~~~
