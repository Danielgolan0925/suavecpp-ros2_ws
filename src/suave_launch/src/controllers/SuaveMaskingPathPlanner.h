//
// Created by patrick on 9/5/24.
//

#ifndef SUAVEMASKINGPATHPLANNER_H
#define SUAVEMASKINGPATHPLANNER_H

#include "../common/ISuaveController.h"
#include "../mavutil/Drone.h"
#include "../mavutil/MavUtil.h"
#include "../common/ITask.h"
#include "../masking_pid/MaskingSubscriber.h"

class SuavePathPlanner: public ISuaveController {
public:
    SuavePathPlanner() :
        ISuaveController()
    {
        try {
            if (const auto& system = connectToPX4(m_mavsdk)) {
                m_drone = std::make_unique<Drone>(system);
            }
        }
        catch (...) { /*ignore*/ }
    }

    void start() override;
    void shutdown() override;

private:
    void start_telemetry_publishing();
    void stop_telemetry_publishing();
    
    bool m_end_controller{ false };
    std::unique_ptr<Drone> m_drone{ nullptr };
    std::vector<std::shared_ptr<ITask>> m_task{};
};

#endif //SUAVEMASKINGPATHPLANNER_H
