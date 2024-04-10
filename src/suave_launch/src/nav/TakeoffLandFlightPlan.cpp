#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>

#include "TakeoffLandFlightPlan.h"

#include <chrono>
#include <thread>

using namespace mavsdk;
using std::this_thread::sleep_for;
using std::chrono::seconds;

FlightPlanResult TakeoffLandFlightPlan::start(std::shared_ptr<mavsdk::System>& system)
{
    // Make sure we have a telemetry plugin
    auto telemetry = Telemetry{system};

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry.set_rate_position(1.0);

    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed:" << set_rate_result << '\n';
        return FlightPlanResult::FAILURE;
    }

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });

    // Make sure we have an action plugin
    auto action = Action{system};

    // Arm the drone
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed:" << arm_result << '\n';
        return FlightPlanResult::FAILURE;
    }

    // Take off
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();

    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed:" << takeoff_result << '\n';
        return FlightPlanResult::FAILURE;
    }

    // Let it hover for a bit before landing again.
    sleep_for(seconds(10));

    // Land the drone
    std::cout << "Landing...\n";
    const Action::Result land_result = action.land();

    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed:" << land_result << '\n';
        return FlightPlanResult::FAILURE;
    }

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(5));
    std::cout << "Finished...\n";

    return FlightPlanResult::SUCCESS;
}
