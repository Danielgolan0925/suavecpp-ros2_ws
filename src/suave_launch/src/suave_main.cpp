#include <csignal>
#include <rclcpp/utilities.hpp>
#include <chrono>
#include <thread>

#include "controllers/SuaveMaskingController.h"
#include "controllers/SuaveVIOTestFlight.h"
#include "controllers/SuaveMaskingPathPlanner.h"
#include "mavutil/Drone.h"

std::unique_ptr<SuavePathPlanner> s_controller;

void signal_handler(int sig)
{
    suave_warn << "Signal received: " << sig << std::endl;
    if (s_controller) {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
    exit(0);
}

int main(int argc, char **argv)
{
    suave_log << "Starting suave_main" << std::endl;
    rclcpp::init(argc, argv);
    std::vector<int> signals = {SIGINT, SIGTERM, SIGQUIT};
    for (const auto sig : signals)
    {
        std::signal(sig, signal_handler);
    }
    try {
        s_controller = std::make_unique<SuavePathPlanner>();
        s_controller->start();
    } catch (const std::exception &e) {
        suave_err << "Caught exception: " << e.what() << std::endl;
    }
    if (s_controller) {
        s_controller->shutdown();
    }
    rclcpp::shutdown();
    suave_log << "Exiting suave_main" << std::endl;
    return 0;
}
