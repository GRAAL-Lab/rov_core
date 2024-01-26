#ifndef ROV_CTRL_NAV_DATA_STRUCTS_H_
#define ROV_CTRL_NAV_DATA_STRUCTS_H_

//#include "ulisse_driver/driver_defines.h"
#include <ctrl_toolbox/HelperFunctions.h>
#include <libconfig.h++>
#include <rclcpp/logging.hpp>

namespace rov {

namespace nav {

    enum class FilterMode : int {
        GroundTruth,
        LuenbergerObserver,
        KalmanFilter
    };

    struct NavigationFilterParams {
        int rate;
        FilterMode mode;
        unsigned int thrusterFIFOdelayLength;

        bool ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
        {
            if (!ctb::GetParam(confObj, rate, "rate"))
                return false;
            //Read the type of filter to use
            int tmp;

            if (!ctb::GetParam(confObj, tmp, "filterMode"))
                return false;
            mode = static_cast<FilterMode>(tmp);

            if (!ctb::GetParam(confObj, thrusterFIFOdelayLength, "thrusterFIFOdelayLength"))
                return false;


            return true;
        }

        friend std::ostream& operator<<(std::ostream& os, NavigationFilterParams const& a)
        {
            return os << "======= NAVIGATION FILTER CONF =======\n"
                      << "Rate: " << a.rate << "\n"
                      //                      << "Gains: " << a.mode << "\n"
                      << "===============================\n";
        }
    };

    enum class CommandType : uint16_t {
        undefined = 0,
        reset = 1,
        reloadconfig = 2,
    };

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };

    std::string CommandTypeToString(CommandType type);
    std::string CommandAnswerToString(CommandAnswer answer);
}
}

#endif /* ROV_CTRL_NAV_DATA_STRUCTS_H_ */
