#include <cstdlib>
#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

#include <yarp/os/Network.h>

#include <CLI/CLI.hpp>

#include <DeviceRegistry.hh>

std::string generateSDFStringThatJustIncludesModel(const std::string &modelUri) {
    std::string xmlContent = R"(
<?xml version="1.0"?>

<sdf version="1.11">
    <world name="gz_sim_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>
        <plugin
            filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact">
        </plugin>

        <include>
            <uri>)" + modelUri + R"(</uri>
        </include>

    </world>
</sdf>
)";

    return xmlContent;
}

int main(int argc, char *argv[])
{
    CLI::App app{"gz-sim-yarp-plugins-check-models"};

    std::string world_file;
    std::string model_uri;

    auto world_option = app.add_option("--world-file", world_file, "Path to the world file (for example ./path/world.sdf)");
    auto model_option = app.add_option("--model-uri", model_uri, "URI to the model (for example model://ergoCub/robots/ergoCubGazeboV1_2)");

    int verbosity_level{0};
    app.add_option("--verbosity", verbosity_level, "Verbosity level passed to gz::common::Console::SetVerbosity (from 0 to 4)")->capture_default_str();

    app.callback([&]() {
        if (world_option->count() == 0 && model_option->count() == 0) {
            throw CLI::ValidationError("You must specify either --world-file or --model-uri");
        }
        if (world_option->count() > 0 && model_option->count() > 0) {
            throw CLI::ValidationError("You cannot specify both --world-file and --model-uri");
        }
    });

    CLI11_PARSE(app, argc, argv);

    // In case the modules include YARP devices that are YARP's Network Wrapper Server devices
    // (i.e. devices that need to read or write from YARP ports), let's select local mode so
    // the test works fine without the need to launch any yarpserver
    yarp::os::NetworkBase::setLocalMode(true);

    gz::common::Console::SetVerbosity(verbosity_level);

    // Object to pass custom configuration to the server
    gz::sim::ServerConfig serverConfig;

    // Populate with some configuration, for example, the SDF file to load
    if (world_option->count() > 0) {
        // It seems that SetSdfFile prever absolute paths
        serverConfig.SetSdfFile(world_file);
    } else if (model_option->count() > 0) {
        // Here we build an empty world with just the inclusion of the specified model
        serverConfig.SetSdfString(generateSDFStringThatJustIncludesModel(model_uri));
    }

    // Instantiate server
    gz::sim::Server server(serverConfig);

    // Run the server paused for 1 iterations blocking to see if the model loads correctly,
    int nrOfIterations = 1;
    server.Run(true /*blocking*/, /*iterations*/ nrOfIterations, /*paused*/false);

    // If there were no iteration, then the server did not load correctly
    if (server.IterationCount() != nrOfIterations)
    {
        return EXIT_FAILURE;
    }

    // At this point. we run the server for one iteration, and we need to know if some plugin failed.
    // As we own this process, we know that there was only one gz-sim server, so we can just check from the gzyarp::DeviceRegistry if there was any device that
    // failed
    if (gzyarp::DeviceRegistry::getHandler()->getTotalNrOfGzSimYARPPluginsNotSuccessfullyLoaded() > 0)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
