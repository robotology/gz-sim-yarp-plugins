# gz-sim-yarp-plugins-check-model

The `gz-sim-yarp-plugins-check-model` is an helper tool that can be used to check automatically if a world or a model containing `gz-sim-yarp-plugins` is able to start correctly, and if all the `gz-sim-yarp-plugins` in it load correctly. It can be useful to automatically check in Continuous Integration if some model or world contained in a given repo (and all its associated configuration file) are able to load.

If you want to check if a world is able to load correctly, you can run the tool as:

~~~
gz-sim-yarp-plugins-check-model --world-file ./local/to/world.sdf
~~~

while to check if a model is able to load correctly, you can pass:

~~~
gz-sim-yarp-plugins-check-model --model-uri model://ergoCubGazeboV1
~~~

The tool will return 0 as exit value if the world or model loaded correctly, or 1 if there was an error in the loading.

For more information and options, run `gz-sim-yarp-plugins-check-model --help`.


## Usage in CI

To run the tool in CI, you can just invoke it in your CI script with the file or model you want to check as:

~~~
gz-sim-yarp-plugins-check-model --model-uri model://ergoCubGazeboV1
~~~

Remember that environment variables such as  `GZ_SIM_RESOURCE_PATH` used to find `model://` files need to be set for the tool to work appropriately.

You can also use the tool as part of a CMake test, with the following CMake code:

~~~cmake
add_test(NAME project-check-world-loads-correctly
         COMMAND gz-sim-yarp-plugins-check-model --world-file ${PROJECT_SOURCE_DIR}/location/of/world/file.sdf)
~~~
