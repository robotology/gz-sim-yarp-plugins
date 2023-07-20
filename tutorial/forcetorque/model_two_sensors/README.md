# Run forcetorque in Gazebo Garden

## Run model with two FT sensors in Gazebo Garden

- 1st terminal:
    ~~~
    yarp server
    ~~~
- 2nd terminal:
    ~~~
    export LIBGL_ALWAYS_SOFTWARE=1 
    cd ../../../build
    export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`
    cd ../tutorial/forcetorque/model_two_sensors
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
    - `yarp read /read /forcetorque1/measures:o`
    ~~~
    () () () () () (((0.0 0.0 0.0 0.0 0.0 0.0) 0.0)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.0389999999999999999445)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.115000000000000004996)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.193000000000000004885)) () () () ()
    () () () () () (((0.0 0.0 -196.0 0.0 0.0 0.0) 0.275000000000000022204)) () () () ()
    ...
    ~~~
    - `yarp read /read /forcetorque2/measures:o`
    ~~~
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.53100000000000058265)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.61299999999999954525)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.68900000000000005684)) () () () ()
    () () () () () (((0.0 0.0 -98.0 0.0 0.0 0.0) 8.77299999999999968736)) () () () ()
    ~~~
