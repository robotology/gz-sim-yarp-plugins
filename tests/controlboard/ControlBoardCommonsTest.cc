#include "../../libraries/common/Common.hh"
#include "../../libraries/singleton-devices/Handler.hh"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <gtest/gtest.h>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/JointForceCmd.hh>

#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

// Checks that the control board can be configured without initial conditions
TEST(ControlBoardCommonsTest, ConfigureControlBoardWithoutInitialCondition)
{
    std::string modelSdfName = "pendulum_no_initial_configuration.sdf";

    gz::sim::TestFixture testFixture{"../../../tests/controlboard/" + modelSdfName};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();
}

// Checks that the control board can be configured without initial conditions
TEST(ControlBoardCommonsTest, ConfigureControlBoardWithInitialCondition)
{
    std::string modelSdfName = "pendulum_with_initial_configuration.sdf";

    gz::sim::TestFixture testFixture{"../../../tests/controlboard/" + modelSdfName};
    gz::common::Console::SetVerbosity(4);

    testFixture.Finalize();
}
