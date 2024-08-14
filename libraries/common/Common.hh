#pragma once

#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace gzyarp
{
constexpr double pi = 3.1415926535897932384626433;

inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * pi;
}

inline double convertRadiansToDegrees(double radians)
{
    return radians * 180.0 / pi;
}

// Convert a degree gain expressed in [Nm/°] to a radian gain expressed in [Nm/rad]
inline double convertDegreeGainToRadianGains(double degreeGain)
{
    return degreeGain * 180.0 / pi;
}

// Convert a radian gain expressed in [Nm/rad] to a degree gain expressed in [Nm/°]
inline double convertRadianGainToDegreeGains(double radianGain)
{
    return radianGain / 180.0 * pi;
}

template <typename T> inline T readElementFromValue(const yarp::os::Value& value);

template <> inline double readElementFromValue<double>(const yarp::os::Value& value)
{
    return value.asFloat64();
}

template <> inline int readElementFromValue<int>(const yarp::os::Value& value)
{
    return value.asInt64();
}

template <> inline std::string readElementFromValue<std::string>(const yarp::os::Value& value)
{
    return value.asString();
}

template <> inline bool readElementFromValue<bool>(const yarp::os::Value& value)
{
    return value.asBool();
}

/**
 * Get a vector from a parameter, using both the recommended style:
 * nameOfList (elem1 elem2 elem3)
 * or the deprecated (since YARP 3.10):
 * nameOfList elem1 elem2 elem3
 *
 *
 * \brief Get vector from YARP configuration
 * \return true if the parsing was successful, false otherwise
 */
template <typename T>
inline bool readVectorFromConfigFile(const yarp::os::Searchable& params,
                                     const std::string& listName,
                                     std::vector<T>& outputList)
{
    bool vectorPopulated = false;
    outputList.resize(0);
    yarp::os::Value& val = params.find(listName);
    if (!val.isNull() && val.isList())
    {
        yarp::os::Bottle* listBot = val.asList();
        outputList.resize(listBot->size());

        for (size_t i = 0; i < outputList.size(); i++)
        {
            outputList[i] = readElementFromValue<T>(listBot->get(i));
        }

        vectorPopulated = true;
    } else
    {
        // Try to interpreter the list via findGroup
        yarp::os::Bottle listBottleAndKey = params.findGroup(listName);
        if (!listBottleAndKey.isNull())
        {
            yWarning() << "Parameter " << listName
                       << " should be a list, but its format is deprecated as parenthesis are "
                          "missing."
                       << " Please add parentesis to the list, as documented in "
                          "https://github.com/robotology/yarp/discussions/3092 .";

            outputList.resize(listBottleAndKey.size() - 1);

            for (size_t i = 0; i < outputList.size(); i++)
            {
                outputList[i] = readElementFromValue<T>(listBottleAndKey.get(i + 1));
            }
            vectorPopulated = true;
        }
    }
    return vectorPopulated;
}

} // namespace gzyarp
