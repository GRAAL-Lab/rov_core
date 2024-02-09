#ifndef ROVDEFINES_H
#define ROVDEFINES_H

/*#include <rml/RMLDefines.h>
#include <vector>*/
#include <string>

namespace rov {

namespace robotModelID {
const std::string blueROV = "BlueROV2";
}

namespace task {

const std::string rovLinearVelocity = "ROV_Linear_Velocity";
const std::string rovAngularVelocity = "ROV_Angular_Velocity";
const std::string rovAngularPosition = "ROV_Angular_Position"; //
const std::string rovAbsoluteAxisAlignment = "ROV_Absolute_Axis_Alignment";
const std::string rovCartesianDistance = "ROV_Cartesian_Distance";

const std::string rovSafetyBoundaries = "ROV_Safety_Boundaries";
const std::string rovAbsoluteAxisAlignmentSafety = "ROV_Absolute_Axis_Alignment_Safety";

const std::string rovAngularPositionHold = "ROV_Angular_Position_Hold"; //
const std::string rovCartesianDistanceHold = "ROV_Cartesian_Distance_Hold";

const std::string rovAbsoluteAxisAlignmentHold = "ROV_Absolute_Axis_Alignment_Hold";
const std::string rovLinearVelocityHold = "ROV_Linear_Velocity_Hold";

}

namespace action {

const std::string goTo = "Move_To";
const std::string halt = "Halt";
const std::string hold = "Hold";
const std::string velocity = "Surge_Sway_Heave_YawRate";
}

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

namespace commands {

namespace ID {

const std::string halt = "halt_command";
const std::string latlongalt = "moveto_command";
const std::string hold = "hold_command";
//const std::string surgeheading = "surgeheading_command";
const std::string velocity = "velocity_command";
//const std::string pathfollow = "pathfollow_command";
}
}

namespace states {

namespace ID {

const std::string halt = "Halt";
const std::string latlongalt = "Move_To";
const std::string hold = "Hold";
//const std::string surgeheading = "Surge_Heading";
const std::string velocity = "Surge_Sway_Heave_YawRate";
//const std::string pathfollow = "Path_Following";
}
}

namespace directions {

namespace ID {

const std::string forward = "Forward";
const std::string backward = "Backward";
const std::string left = "Left";
const std::string right = "Right";
const std::string turn_left = "Turn_Left";
const std::string turn_right = "Turn_Right";
const std::string up = "Up";
const std::string down = "Down";


}
}

namespace events {

namespace names {
const char* const neargoalposition = "NEARGOALPOSITION";
const char* const switchstate = "SWITCHSTATE";
//const char* const surgeheadingtimeout = "SURGEHEADINGTIMEOUT";
//const char* const surgeyawratetimeout = "SURGEYAWRATETIMEOUT";
const char* const rcenabled = "RCENABLED";
}

namespace topicnames {
const char* const events = "/ctrl/out/events";
}

namespace priority {
const uint8_t high = 10;
const uint8_t medium = 5;
const uint8_t low = 1;
}
}


}
#endif
