#ifndef ROVDEFINES_H
#define ROVDEFINES_H

/*#include <rml/RMLDefines.h>
#include <vector>*/
#include <string>

namespace rov {

namespace robotModelID {
const std::string ROV = "BlueROV2";
}

/*namespace task {

const std::string asvLinearVelocity = "ASV_Linear_Velocity";
const std::string asvAngularPosition = "ASV_Angular_Position";
const std::string asvAngularPositionHold = "ASV_Angular_Position_Hold";
const std::string asvAbsoluteAxisAlignment = "ASV_Absolute_Axis_Alignment";
const std::string asvCartesianDistance = "ASV_Cartesian_Distance";
const std::string asvCartesianDistanceHold = "ASV_Cartesian_Distance_Hold";
const std::string asvCartesianDistancePathFollowing = "ASV_Cartesian_Distance_Path_Follow";
const std::string asvSafetyBoundaries = "ASV_Safety_Boundaries";
const std::string asvAbsoluteAxisAlignmentSafety = "ASV_Absolute_Axis_Alignment_Safety";
const std::string asvAbsoluteAxisAlignmentHold = "ASV_Absolute_Axis_Alignment_Hold";
const std::string asvLinearVelocityHold = "ASV_Linear_Velocity_Hold";

}
*/
/*
namespace action {

const std::string goTo = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string surge_heading = "Surge_Heading";
const std::string surge_yawrate = "Surge_YawRate";
const std::string pathfollow = "Path_Following";
}
*/
namespace inputs {

namespace ID {

const uint8_t halt = 0;
const uint8_t hold = 5;
const uint8_t forward = 8;
const uint8_t backward = 2;
const uint8_t left = 6;
const uint8_t right = 4;
const uint8_t up = 9;
const uint8_t down = 3;
const uint8_t turn_left = 7;
const uint8_t turn_right = 1;
}
}

namespace states {

namespace ID {

//const std::string latlong = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string forward = "Forward";
const std::string backward = "Backward";
const std::string left = "Left";
const std::string right = "Right";
const std::string turn_left = "Turn_Left";
const std::string turn_right = "Turn_Right";
const std::string up = "Up";
const std::string down = "Down";
//const std::string surgeheading = "Surge_Heading";
const std::string surgeyawrate = "Surge_YawRate";
//const std::string pathfollow = "Path_Following";
}
}


/*
namespace events {

namespace names {
const char* const neargoalposition = "NEARGOALPOSITION";
const char* const switchstate = "SWITCHSTATE";
//const char* const surgeheadingtimeout = "SURGEHEADINGTIMEOUT";
//const char* const surgeyawratetimeout = "SURGEYAWRATETIMEOUT";
const char* const rcenabled = "RCENABLED";
}
*/

/*
namespace topicnames {
const char* const events = "/ctrl/out/events";
}
*/
namespace priority {
const uint8_t high = 10;
const uint8_t medium = 5;
const uint8_t low = 1;
}


}
#endif
