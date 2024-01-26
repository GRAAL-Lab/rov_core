
#include "nav_filter_rov/nav_data_structs.hpp"

namespace rov {

namespace nav {


    std::string CommandTypeToString(CommandType type)
    {
        std::string name;
        uint16_t type_uint = static_cast<uint16_t>(type); //new type of cast in C++11

        switch (type_uint) {
        case (uint16_t)CommandType::undefined:
            name = "undefined";
            break;
        case (uint16_t)CommandType::reset:
            name = "reset";
            break;
        case (uint16_t)CommandType::reloadconfig:
            name = "reloadconfig";
            break;
        default:
            name = "Unhandled...please update ulisse::nav::CommandTypeToString method adding command type=" + std::to_string(type_uint);
            break;
        }

        return name;
    }


    std::string CommandAnswerToString(CommandAnswer answer)
    {
        std::string name;
        int16_t ans_int = (int16_t)answer;

        switch (ans_int) {
        case (int16_t)CommandAnswer::fail:
            name = "fail";
            break;
        case (int16_t)CommandAnswer::undefined:
            name = "undefined";
            break;
        case (int16_t)CommandAnswer::ok:
            name = "ok";
            break;
        default:
            name = "Unhandled...please update ulisse::nav::CommandAnswerToString method adding answer type=" + std::to_string(ans_int);
            break;
        }

        return name;
    }
}
}
