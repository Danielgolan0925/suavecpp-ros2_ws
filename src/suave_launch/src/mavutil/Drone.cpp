//
// Created by suave on 4/17/24.
//

#include "Drone.h"
#include <cmath>
#include <sstream>

#include "../controllers/ControllerMacros.h"

Drone::Drone(std::shared_ptr<System> system): m_system(std::move(system))
{
    m_initial_heading_rad = 0;
    suave_log << "Initial heading: " << m_initial_heading_rad << " rad\n";
}
std::string Drone::get_quaternion_string() const
{
    auto quaternion = get_quaternion();
    std::stringstream ss;
    ss << "Quaternion- w: " << quaternion.w << ", x: " << quaternion.x << ", y: " << quaternion.y << ", z: " << quaternion.z;
    return ss.str();
}

std::string Drone::get_ned_position_string() const
{
    auto position_velocity_ned = m_position_velocity_ned.get().unwrap();
    auto position = position_velocity_ned.position;
    std::stringstream ss;
    ss << "NED Position- North: " << position.north_m << ", East: " << position.east_m << ", Down: " << position.down_m;
    return ss.str();
}

Offboard::Result Drone::offboard_setpoint()
{
    return offboard().set_velocity_body({0, 0, 0, 0});
}

Offboard::Result Drone::set_relative_position_ned(const float& n, const float& e, const float& d)
{
    const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
    const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
    return offboard().set_position_ned({north_m + n, east_m + e, down_m + d, yaw});
}

void Drone::set_local_position_setpoint()
{
    const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
    const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
    m_local_setpoint = { north_m, east_m, down_m, yaw };
}

Offboard::Result Drone::set_local_position_ned(double n, double e, double d)
{
    if (!m_local_setpoint)
    {
        const auto [north_m, east_m, down_m] = m_position_velocity_ned.wait_for_next().unwrap().position;
        const auto yaw = m_attitude_euler.wait_for_next(1).unwrap().yaw_deg;
        m_local_setpoint = { north_m, east_m, down_m, yaw };
    }
    return offboard().set_position_ned({ m_local_setpoint->north + n, m_local_setpoint->east + e, m_local_setpoint->down + d, m_local_setpoint->yaw });
}

Offboard::Result Drone::offboard_land()
{
    const auto yaw = m_attitude_euler.wait_for_next().unwrap().yaw_deg;
    return offboard().set_velocity_ned({0, 0, 0.5, yaw});
}

Offboard::Result Drone::offboard_wait_for_land()
{
    auto land_result = offboard_land();
    if (land_result != Offboard::Result::Success)
    {
        return land_result;
    }

    // Wait for drone to land
    int elapse_sec = 0;
    constexpr auto MAX_ELAPSE = 10;
    while (in_air())
    {
        suave_log << "Waiting for drone to land, elapse_sec: " << elapse_sec << std::endl;

        if (elapse_sec >= MAX_ELAPSE)
        {
            break;
        }

        sleep(1)
        elapse_sec++;
    }
    sleep(3)
    auto disarm_result = action().disarm();
    if (disarm_result != Action::Result::Success)
    {
        suave_err << "Disarm failed: " << disarm_result << std::endl;
    }

    return land_result;
}

Offboard::Result Drone::offboard_hold()
{
    return set_relative_position_ned(0, 0, 0);
}

Tune::Result Drone::play_waiting_tune()
{
    Tune::SongElement note = Tune::SongElement::NoteC;
    Tune::TuneDescription waiting_tune{{note, note, note}, 90};
    return tune().play_tune(waiting_tune);
}

Tune::Result Drone::play_ready_tune()
{
    Tune::SongElement c = Tune::SongElement::NoteC;
    Tune::SongElement d = Tune::SongElement::NoteD;
    Tune::SongElement e = Tune::SongElement::NoteE;
    Tune::SongElement b = Tune::SongElement::NoteB;
    Tune::TuneDescription ready_tune{{c, d, e, b}, 90};
    return tune().play_tune(ready_tune);
}

void Drone::set_heading_callback(TelemetryProperty<Telemetry::Heading>::TCallback callback)
{
    m_heading.set_callback(callback);
}
