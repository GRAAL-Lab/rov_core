#ifndef ROV_CONFIGURATION_H
#define ROV_CONFIGURATION_H

#include <ikcl/ikcl.h>
#include <rov_ctrl/ctrl_data_structs.hpp>
#include <rov_ctrl/states/generic_state.hpp>

bool ConfigureTasksFromFile(std::unordered_map<std::string, rov::TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, rov::TasksInfo>& tasksMap, libconfig::Config& confObj);
bool ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj);
bool ConfigureSatesFromFile(std::unordered_map<std::string, std::shared_ptr<rov::states::GenericState>> statesMap, libconfig::Config& confObj);
#endif // ROV_CONFIGURATION_H
