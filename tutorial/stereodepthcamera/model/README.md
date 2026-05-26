# Run Stereo Depth Camera Plugin in Gazebo

This example creates two RGBD sensors separated by a 10 cm baseline. The
plugin registers a YARP RGBD device for each eye and publishes an optional
side-by-side RGB stream:

- `/stereodepthcamera/left/rgbImage:o` and `/stereodepthcamera/left/depthImage:o`
- `/stereodepthcamera/right/rgbImage:o` and `/stereodepthcamera/right/depthImage:o`
- `/stereodepthcamera/rgbImage:o`, containing left and right RGB frames side by side

## Run the Model

1. Start the YARP server:

   ```sh
   yarp server
   ```

2. In another terminal, make the tutorial resources and built plugins
   discoverable, then launch Gazebo:

   ```sh
   export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:<path-to-repo>/tutorial"
   export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:<path-to-repo>/build/lib"
   cd <path-to-repo>/tutorial/stereodepthcamera/model
   gz sim model.sdf
   ```

3. Start three RGB viewers:

   ```sh
   yarpview --name /stereo_left &
   yarpview --name /stereo_right &
   yarpview --name /stereo_pair &
   yarp connect /stereodepthcamera/left/rgbImage:o /stereo_left fast_tcp
   yarp connect /stereodepthcamera/right/rgbImage:o /stereo_right fast_tcp
   yarp connect /stereodepthcamera/rgbImage:o /stereo_pair fast_tcp
   ```

4. To visualize depth as an RGB image, open an additional viewer and connect
   one of the depth streams through YARP's depth image monitor:

   ```sh
   yarpview --name /stereo_depth &
   yarp connect /stereodepthcamera/left/depthImage:o /stereo_depth fast_tcp+recv.portmonitor+type.dll+file.depthimage_to_rgb
   ```

Finally, press Play in Gazebo. The `/stereo_pair` viewer displays the left eye
in its left half and the right eye in its right half.

## Plugin Configuration

The plugin configuration in `model.sdf` associates the two Gazebo RGBD
sensors with two device registry entries used by the network wrappers:

```xml
<yarpConfigurationString>
  (parentLinkName camera_link)
  (leftSensorName left_sensor)
  (rightSensorName right_sensor)
  (leftYarpDeviceName stereodepthcamera_left_plugin_device)
  (rightYarpDeviceName stereodepthcamera_right_plugin_device)
  (stereoRgbPortName /stereodepthcamera/rgbImage:o)
</yarpConfigurationString>
```

`stereoRgbPortName` is optional. Remove it when only the two RGBD devices are
required.

## Run the Automated Test

The automated `StereoDepthCameraTest` is independent from the manual example
above:

- Do not start `yarp server`: the test enables YARP local mode internally.
- Do not start `gz sim`: the test creates and runs a Gazebo server from
  `tests/stereodepthcamera/model.sdf` through `gz::sim::TestFixture`.
- Do not start `yarpview`: the test opens its own RGBD clients and checks the
  received images programmatically.

Configure and build the test:

```sh
cd <path-to-repo>
cmake -S . -B build -DBUILD_TESTING:BOOL=ON
cmake --build build --target gz-sim-yarp-stereodepthcamera-system StereoDepthCameraTest -j4
```

If CMake selects GTest from a Conda environment and linking fails against
ROS/Gazebo dependencies, configure with the system GTest explicitly:

```sh
cmake -S . -B build -DBUILD_TESTING:BOOL=ON \
  -DGTest_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTest
```

Execute only this test:

```sh
ctest --test-dir build --output-on-failure -R '^StereoDepthCameraTest$'
```

The test checks that both RGBD eyes provide RGB and depth frames, then checks
that `/stereodepthcamera/rgbImage:o` contains the two RGB images side by side.

The test still requires a working Gazebo rendering backend: RGBD frames are
rendered even though no Gazebo graphical application is opened. Errors from
`Ogre2RenderEngine` or `eglInitialize` indicate a GPU/display/EGL setup
problem, not a missing `yarp server` or a missing manually launched `gz sim`.
The existing `DepthCameraTest` can be used as a control check:

```sh
ctest --test-dir build --output-on-failure -R '^DepthCameraTest$'
```
