# Run Laser Plugin in Gazebo

## Run model in Gazebo with YARP integration

- 1st terminal:
  ~~~
  yarp server
  ~~~
- 2nd terminal:
  ~~~
  cd tutorial/laser
  gz sim model.sdf
  ~~~
- 3rd terminal:
  ~~~
  yarp name list
  yarp read /read /laser
  ~~~

Finally start the simulation in Gazebo. If we put an obstacle (for example a cylinder) in front of the laser sensor in the Gazebo simulation, the corresponding output is:
~~~
0.0 360.0 0.100000000000000005551 5.0 (1.11125278472900390625 1.10634744167327880859 1.10667657852172851562 1.11030185222625732422 1.10864961147308349609 1.12173914909362792969 1.11585438251495361328 1.11526703834533691406 1.12902033329010009766 1.12489259243011474609 1.12953507900238037109 1.13095843791961669922 1.13082158565521240234 1.13813734054565429688 1.15377604961395263672 1.14405727386474609375 1.15732824802398681641 1.15743625164031982422 1.16749060153961181641 1.19045126438140869141 1.18509876728057861328 1.19290518760681152344 1.19264888763427734375 1.21191620826721191406 1.21245372295379638672 inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf inf 1.21789622306823730469 1.19324958324432373047 1.19849312305450439453 1.18799138069152832031 1.178630828857421875 1.16958940029144287109 1.16730511188507080078 1.15926289558410644531 1.14865171909332275391 1.14918661117553710938 1.13976693153381347656 1.13293814659118652344 1.13386213779449462891 1.13239908218383789062 1.12463426589965820312 1.11819624900817871094 1.11554396152496337891 1.11691248416900634766 1.1094226837158203125 1.11155080795288085938 1.11591613292694091797 1.11988699436187744141 1.11187469959259033203 1.12193655967712402344) 1
...
~~~

If you are using Linux on WSLg and you have the following error
~~~
OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo at ./RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 685)
~~~
try forcing software rendering:
~~~
export LIBGL_ALWAYS_SOFTWARE=1
~~~

## Run model in Gazebo without YARP integration
Run the model:
```
gz sim model.sdf
```
List all available Gazebo topics:
```
gz topic -l
```
Print the topic that correspond to the Laser sensor, in this case:
```
gz topic -e -t <laser_topic_name>
```
where you should substitute `<laser_topic_name>` with the topic of the Laser name (in this case  `gz topic -e -t /world/sensors/model/laser_model/link/link_1/sensor/laser_sensor/scan`).    
Youl should see an output like this:
```
header {
  stamp {
    sec: 1
    nsec: 552000000
  }
  data {
    key: "frame_id"
    value: "laser_model::link_1::laser_sensor"
  }
  data {
    key: "seq"
    value: "46"
  }
}
frame: "laser_model::link_1::laser_sensor"
world_pose {
  position {
  }
  orientation {
    w: 1
  }
}
angle_min: -3.14159
angle_max: 3.14159
angle_step: 0.017501894150417828
range_min: 0.1
range_max: 5
count: 360
vertical_angle_step: nan
vertical_count: 1
ranges: 1.1112527847290039
ranges: 1.1063474416732788
ranges: 1.1066765785217285
ranges: 1.1103018522262573
ranges: 1.1086496114730835
ranges: 1.1217391490936279
ranges: 1.1158543825149536
ranges: 1.1152670383453369
ranges: 1.1290203332901
ranges: 1.1248925924301147
ranges: 1.1295350790023804
ranges: 1.1309584379196167
ranges: 1.1308215856552124
ranges: 1.1381373405456543
ranges: 1.1537760496139526
ranges: 1.1440572738647461
ranges: 1.1573282480239868
ranges: 1.1574362516403198
ranges: 1.1674906015396118
ranges: 1.1904512643814087
ranges: 1.1850987672805786
ranges: 1.1929051876068115
ranges: 1.1926488876342773
ranges: 1.2119162082672119
ranges: 1.2124537229537964
ranges: inf
ranges: inf
ranges: inf
ranges: inf
ranges: inf
ranges: inf
...
```


