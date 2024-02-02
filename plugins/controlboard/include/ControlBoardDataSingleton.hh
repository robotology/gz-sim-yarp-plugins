#pragma once

#include <array>
#include <gz/common/Event.hh>
#include <mutex>
#include <string>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Stamp.h>

struct ControlBoardData
{
    std::mutex mutex;
    std::string modelScopedName;
};

namespace gzyarp
{

class ControlBoardDataSingleton
{

public:
    static ControlBoardDataSingleton* getControlBoardHandler();

    bool setControlBoardData(ControlBoardData* _controlBoardPtr);

    ControlBoardData* getControlBoardData(const std::string& _controlBoardScopedName) const;

    void removeControlBoard(const std::string& _controlBoardScopedName);

private:
    ControlBoardDataSingleton();
    static ControlBoardDataSingleton* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, ControlBoardData*> ControlBoardMap;
    ControlBoardMap m_controlBoardMap; // map of control boards with their scoped name as key
};

} // namespace gzyarp
