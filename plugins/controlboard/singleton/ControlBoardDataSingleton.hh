#pragma once

#include <ControlBoardData.hh>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Event.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/os/Stamp.h>

namespace gzyarp
{

class ControlBoardDataSingleton
{

public:
    static ControlBoardDataSingleton* getControlBoardHandler();

    bool setControlBoardData(ControlBoardData* _controlBoardPtr);

    ControlBoardData* getControlBoardData(const std::string& _controlBoardScopedName) const;

    void removeControlBoard(const std::string& _controlBoardScopedName);

    std::vector<std::string> getControlBoardKeys() const;

private:
    ControlBoardDataSingleton();
    static ControlBoardDataSingleton* s_handle;
    static std::mutex& mutex();
    typedef std::map<std::string, ControlBoardData*> ControlBoardMap;
    ControlBoardMap m_controlBoardMap; // map of control boards with their scoped name as key
};

} // namespace gzyarp
