//
// Created by suave on 4/17/24.
//

#ifndef DRONE_H
#define DRONE_H
#include <system.h>
#include <thread>
#include <plugins/action/action.h>
#include <tuple>
#include <optional>

#include <plugins/offboard/offboard.h>
#include <plugins/telemetry/telemetry.h>
#include <plugins/param/param.h>
#include <plugins/tune/tune.h>

#include "TelemetryProperty.h"
#include "../util/Util.h"

using namespace mavsdk;

class Drone {
public:
    explicit Drone(std::shared_ptr<System> system);

    std::shared_ptr<System> system() { return m_system; }
    Action& action() { return m_action; }
    Offboard& offboard() { return m_offboard; }
    Telemetry& telemetry() { return m_telemetry; }
    Param& param() { return m_param; }
    Tune& tune() { return m_tune; }

    [[nodiscard]]
    double initial_heading_rad() const { return m_initial_heading_rad; }

    // Telemetry Properties
    [[nodiscard]]
    double heading_rad() const
    {
        return heading_deg() * M_PI / 180;
    }
    [[nodiscard]]
    double heading_deg() const
    {
        return m_heading.get().unwrap().heading_deg;
    }
    [[nodiscard]]
    bool in_air() const
    {
        return m_in_air.get().unwrap();
    }
    [[nodiscard]]
    Telemetry::Quaternion get_quaternion() const
    {
        return m_quaternion.get().unwrap();
    }

    void print_quaternion() const;
    void print_ned_position() const;

    Offboard::Result offboard_setpoint();

    // Blocking until next position update
    Offboard::Result set_relative_position_ned(const float& n, const float& e, const float& d);
    void set_local_position_setpoint();
    Offboard::Result set_local_position_ned(double n, double e, double d);  

    Offboard::Result offboard_land();
    Offboard::Result offboard_wait_for_land();

    Offboard::Result offboard_hold();

    // Tunes
    Tune::Result play_waiting_tune();
    Tune::Result play_ready_tune();

    void set_heading_callback(TelemetryProperty<Telemetry::Heading>::TCallback callback);

private:
    // Mavsdk
    std::shared_ptr<System> m_system{};
    Action m_action{*m_system};
    Offboard m_offboard{*m_system};
    Telemetry m_telemetry{*m_system};
    Param m_param{*m_system};
    Tune m_tune{*m_system};

    // Telemetry Properties
    TelemetryProperty<Telemetry::Heading> m_heading {m_telemetry, &Telemetry::subscribe_heading};
    TelemetryProperty<bool> m_in_air {m_telemetry, &Telemetry::subscribe_in_air};
    TelemetryProperty<Telemetry::PositionVelocityNed> m_position_velocity_ned {m_telemetry, &Telemetry::subscribe_position_velocity_ned};
    TelemetryProperty<Telemetry::EulerAngle> m_attitude_euler {m_telemetry, &Telemetry::subscribe_attitude_euler};
    TelemetryProperty<Telemetry::Quaternion> m_quaternion {m_telemetry, &Telemetry::subscribe_attitude_quaternion};

    // Misc
    double m_initial_heading_rad{ NAN };
    struct LocalPosition 
    {
        float north; 
        float east; 
        float down;
        float yaw;
    };
    std::optional<LocalPosition> m_local_setpoint{};
};

#endif //DRONE_H
