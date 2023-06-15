#include <yarp/robotinterface/XMLReader.h>
#include <yarp/os/Network.h>
#include <iostream>


int main() {
    yarp::os::Network network;
    std::string pathToXmlConfigurationFile = "prova.xml";
    yarp::robotinterface::XMLReader reader;
    yarp::robotinterface::XMLReaderResult result = reader.getRobotFromFile(pathToXmlConfigurationFile);
    
    if (!result.parsingIsSuccessful) {
        std::cerr << "Error parsing XML file." << std::endl;
        return 1;
    }
    
    // Enter the startup phase, that will open all the devices and  call attach if necessary
    bool ok = result.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup);
    
    if (!ok) {
        std::cerr << "Error entering startup phase." << std::endl;
        return 1;
    }
    
    // At this point, the system is running and the thread that called the
    // enterPhase methods does not need to do anything else
    
    // This code needs to be executed when you want to close the robot
    // Close robot, that will close all the devices and  call detach if necessary
    bool shutdownRequested = false;
    while (!shutdownRequested) {
        std::string userInput;
        std::cout << "Enter 'x' to initiate shutdown: ";
        std::cin >> userInput;
        if (userInput == "x") {
            ok = result.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1);
            ok = ok && result.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown);

            if (!ok) {
                std::cerr << "Error entering shutdown phase." << std::endl;
                return 1;
            }
            shutdownRequested = true;
        }
    }
}

