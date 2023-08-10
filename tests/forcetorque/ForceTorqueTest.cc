#include <gtest/gtest.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <gz/sim/TestFixture.hh>

TEST(ForceTorqueTest, PluginTest)
{
  yarp::os::NetworkBase::setLocalMode(true);
  
  // Maximum verbosity helps with debugging
  gz::common::Console::SetVerbosity(4);

  // Instantiate test fixture
  gz::sim::TestFixture fixture("../../../tests/forcetorque/model.sdf");

  int iterations = 1000;
  fixture.Server()->Run(true, iterations, false);

  yarp::os::Port p;         
  p.open("/read");  
  yarp::os::Network::connect("/forcetorque/measures:o","/read");
  yarp::os::Bottle b;        
  p.read(b);         
  std::stringstream ss(b.get(5).toString());

  char parenthesis;
  double forceX, forceY, forceZ, torqueX, torqueY, torqueZ, simTime;

  ss >> parenthesis >> parenthesis >>
        forceX >> forceY >> forceZ >> 
        torqueX >> torqueY >> torqueZ >> 
        parenthesis >> simTime ;


  EXPECT_NEAR(forceX, 0.0, 1e-2);
  EXPECT_NEAR(forceY, 0.0, 1e-2);
  EXPECT_NEAR(forceZ, -9.8*10, 1e-2);
  EXPECT_NEAR(torqueX, 0.0, 1e-2);
  EXPECT_NEAR(torqueY, 0.0, 1e-2);
  EXPECT_NEAR(torqueZ, 0.0, 1e-2);
  EXPECT_GT(simTime, 0.0);
}
