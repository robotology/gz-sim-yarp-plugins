macro(gsyp_add_tutorial_model_check tutorialname worldrelpath)
    add_test(NAME gz-sim-yarp-plugins-check-model-tutorial-${tutorialname}
             COMMAND gz-sim-yarp-plugins-check-model --world-file ${PROJECT_SOURCE_DIR}/tutorial/${worldrelpath})
    set(_env_vars)
    list(APPEND _env_vars
        "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>"
        "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${PROJECT_SOURCE_DIR}/tutorial"
    )
    set_tests_properties(gz-sim-yarp-plugins-check-model-tutorial-${tutorialname} PROPERTIES
        ENVIRONMENT "${_env_vars}")
endmacro()

# Check that the tutorial starts correctly
gsyp_add_tutorial_model_check(basestate basestate/model.sdf)
# TODO(traversaro): test that are comment are segfaulting on Linux, we should understand why and uncomment them
# gsyp_add_tutorial_model_check(camera-model camera/model/model.sdf)
# gsyp_add_tutorial_model_check(camera-model-horizontal-flip camera/model_horizontal_flip/model.sdf)
# gsyp_add_tutorial_model_check(camera-model-vertical-flip camera/model_vertical_flip/model.sdf)
# gsyp_add_tutorial_model_check(laser laser/model.sdf)
gsyp_add_tutorial_model_check(clock clock/model.sdf)
gsyp_add_tutorial_model_check(forcetorque-model-one-sensor forcetorque/model_one_sensor/model.sdf)
gsyp_add_tutorial_model_check(forcetorque-model-two-sensors forcetorque/model_two_sensors/model2sensors.sdf)
gsyp_add_tutorial_model_check(single-pendulum single_pendulum/model.sdf)

# Then check that the command correctly exit a failure if we pass it a invalid model
add_test(NAME gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file
         COMMAND gz-sim-yarp-plugins-check-model --world-file  ./world_with_robotinterface_with_non_existing_filename.sdf
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(_env_vars)
list(APPEND _env_vars
    "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>"
)
set_tests_properties(gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file PROPERTIES
    ENVIRONMENT "${_env_vars}")
set_property(TEST gz-sim-yarp-plugins-check-model-fails-as-expected-with-wrong-world-file PROPERTY WILL_FAIL true)

# Check that the command fails if not argument is passed
add_test(NAME gz-sim-yarp-plugins-check-model-fails-as-expected-with-no-options
         COMMAND gz-sim-yarp-plugins-check-model)
set_property(TEST gz-sim-yarp-plugins-check-model-fails-as-expected-with-no-options PROPERTY WILL_FAIL true)
