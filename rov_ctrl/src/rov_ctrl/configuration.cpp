#include <libconfig.h++>
#include "rov_ctrl/configuration.hpp"
#include "rov_ctrl/rov_defines.hpp"

bool ConfigureTasksFromFile(std::unordered_map<std::string, rov::TasksInfo>& tasksMap, libconfig::Config& confObj)
{
    for (auto& map : tasksMap) {
        if (!map.second.task->ConfigFromFile(confObj)) {
            std::cerr << "Failed to configure task " << map.first << " form file" << std::endl;
            return false;
        }
    }

    return true;
}

bool ConfigurePriorityLevelsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, std::unordered_map<std::string, rov::TasksInfo>& tasksMap, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& priorityLevels = root["priorityLevels"];

    std::vector<std::string> hierarchy;

    for (int i = 0; i < priorityLevels.getLength(); ++i) {

        const libconfig::Setting& priorityLevel = priorityLevels[i];
        std::string PLID;

        if (!ctb::GetParam(priorityLevel, PLID, "name"))
            return false;

        hierarchy.push_back(PLID);

        //configure regularization data
        rml::RegularizationData regularizationData;

        if (!ctb::GetParam(priorityLevel, regularizationData.params.lambda, "lambda"))
            return false;
        if (!ctb::GetParam(priorityLevel, regularizationData.params.threshold, "threshold"))
            return false;

        actionManager->AddPriorityLevelWithRegularization(PLID, regularizationData);

        //add tasks to PL
        const libconfig::Setting& tasks = priorityLevel["tasks"];
        for (int i = 0; i < tasks.getLength(); ++i) {
            libconfig::Setting& task = tasks[i];

            std::cout << "Added task: " << task.c_str() << " to " << PLID << std::endl;

            std::unordered_map<std::string, rov::TasksInfo>::iterator it = tasksMap.find(task.c_str());

            actionManager->AddTaskToPriorityLevel(it->second.task, PLID);
        }
    }
    //Set the hierarchy
    actionManager->SetUnifiedHierarchy(hierarchy);

    return true;
}

bool ConfigureActionsFromFile(std::shared_ptr<tpik::ActionManager> actionManager, libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& actions = root["actions"];

    std::vector<std::string> actionPL;

    for (int i = 0; i < actions.getLength(); ++i) {

        const libconfig::Setting& action = actions[i];
        std::string actionID;
        if (!ctb::GetParam(action, actionID, "name"))
            return false;

        std::vector<std::string> actionPL;

        const libconfig::Setting& priorityLevels = action["levels"];
        for (int i = 0; i < priorityLevels.getLength(); ++i) {

            libconfig::Setting& priorityLevel = priorityLevels[i];

            actionPL.push_back(priorityLevel.c_str());
        }

        try {
            actionManager->AddAction(actionID, actionPL);
            std::cout << "Added: " << actionID << " action with PLs:" << std::endl;
            for (auto& pl : actionPL)
                std::cout << pl << std::endl;

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "Configuration Action Manager Exception:" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }

    return true;
}

bool ConfigureSatesFromFile(std::unordered_map<std::string, std::shared_ptr<rov::states::GenericState>> statesMap, libconfig::Config& confObj)
{
    for (auto& mapLine : statesMap) {
        if (!mapLine.second->ConfigureStateFromFile(confObj)) {
            std::cerr << "Failed to configure from file: " << mapLine.first << std::endl;
            return false;
        }
    }
    return true;
}
