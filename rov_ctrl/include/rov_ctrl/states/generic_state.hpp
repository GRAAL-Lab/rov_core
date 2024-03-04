#ifndef ROV_CTRL_GENERIC_STATE_HPP
#define ROV_CTRL_GENERIC_STATE_HPP

#include "rov_ctrl/ctrl_data_structs.hpp"
#include <fsm/fsm.h>
#include <ikcl/ikcl.h>
#include <libconfig.h++>

//to be delete once safetyB in ikcl
#include "rov_ctrl/tasks/SafetyBoundaries.hpp"

namespace rov {

namespace states {

    class GenericState : public fsm::BaseState {
    protected:
        std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask_;

        double minHeadingError_, maxHeadingError_, minZalignmentError_, maxZalignmentError_;

    public:
        std::shared_ptr<ControlData> ctrlData; // [x y]
        std::shared_ptr<tpik::ActionManager> actionManager;
        std::shared_ptr<rml::RobotModel> robotModel;
        std::unordered_map<std::string, TasksInfo> tasksMap;

        GenericState();
        virtual ~GenericState(void);

        void CheckRadioController();
        virtual bool ConfigureStateFromFile(libconfig::Config& confObj) = 0;
    };
}
}

#endif // ROV_CTRL_GENERIC_STATE_HPP
