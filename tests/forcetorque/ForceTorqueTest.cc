#include <gtest/gtest.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
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
  fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);

  yarp::os::Property option;
  option.put("device","multipleanalogsensorsclient");
  option.put("remote","/forcetorque");
  option.put("timeout",1.0);
  option.put("local", "/ForceTorqueTest");
  yarp::dev::PolyDriver driver;

  ASSERT_TRUE(driver.open(option));

  yarp::dev::ISixAxisForceTorqueSensors* isixaxis = nullptr;

  ASSERT_TRUE(driver.view(isixaxis));

  fixture.Server()->Run(/*_blocking=*/true, iterations, /*_paused=*/false);
  yarp::sig::Vector measure(6);
  std::string sensorName;
  double timestamp;
  
  isixaxis->getSixAxisForceTorqueSensorName(0, sensorName);
  isixaxis->getSixAxisForceTorqueSensorMeasure(0, measure, timestamp);
  //std::cerr << "The measure of FT sensor " << sensorName << " is " << measure.toString() << " at time " << timestamp << std::endl;

  EXPECT_NEAR(measure(0), 0.0, 1e-2);
  EXPECT_NEAR(measure(1), 0.0, 1e-2);
  EXPECT_NEAR(measure(2), -9.8*10, 1e-2);
  EXPECT_NEAR(measure(3), 0.0, 1e-2);
  EXPECT_NEAR(measure(4), 0.0, 1e-2);
  EXPECT_NEAR(measure(5), 0.0, 1e-2);
  EXPECT_GT(timestamp, 0.0);
}
