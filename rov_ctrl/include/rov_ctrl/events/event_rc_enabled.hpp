
#ifndef SRC_CTRL_EVENTS_EVENTRCENABLED_H_
#define SRC_CTRL_EVENTS_EVENTRCENABLED_H_

#include "fsm/fsm.h"

namespace rov {

namespace events {

    class EventRCEnabled : public fsm::BaseEvent {
    public:
        EventRCEnabled(void) {}
        virtual ~EventRCEnabled(void) {}
        virtual fsm::retval Execute(void);
        virtual fsm::retval Propagate(void);
    };

} // namespace events

} // namespace rov

#endif /* SRC_CTRL_EVENTS_EVENTRCENABLED_H_ */
