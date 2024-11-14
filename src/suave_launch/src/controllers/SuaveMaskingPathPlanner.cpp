//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingPathPlanner.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"
#include "ControllerMacros.h"

void SuavePathPlanner::start() {
    // Create realsense and rtabmap nodes

    auto realsense_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true"
        }
    );
    m_task.push_back(realsense_task);

    auto rtabmap_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "ros2 param set /camera/camera depth_module.emitter_enabled 0",
            "ros2 launch ~/Dev/suavecpp-ros2_ws/launch/suave_slam.py"
        }
    );
    m_task.push_back(rtabmap_task);

    // Create ROS spinner and add vio bridge and cloud exporter node
    auto vio_spinner = std::make_shared<RosNodeSpinner>();
    m_task.push_back(vio_spinner);

    const auto vio_bridge_node = std::make_shared<VIOBridge>(m_drone->system(), m_drone->initial_heading_rad());
    vio_spinner->add_node(vio_bridge_node);

    const auto cloud_exporter_node = std::make_shared<CloudExporter>();
    vio_spinner->add_node(cloud_exporter_node);

    suave_log << "Starting SLAM" << std::endl;

    realsense_task->start_in_thread();
    sleep(3);
    rtabmap_task->start_in_thread();

    suave_log << "Waiting for rtabmap to initialize..." << std::endl;

    sleep(5);

    suave_log << "Move the drone around to create initial map" << std::endl;
    std::cin.clear(); // need to clear cin before calling await_confirmation, maybe fix this later...
    await_confirmation;
    await_confirmation;

    suave_log << "Starting VIO" << std::endl;
    vio_spinner->start_in_thread();

    suave_log << "Ready for flight?" << std::endl;
    await_confirmation;

    // Start offboard and arm
    try_action(m_drone->action().arm())
    try_offboard(m_drone->offboard_setpoint())
    try_offboard(m_drone->offboard().start())

    while (true) 
    {
        suave_log << "Input action: ";
        std::string buffer{};
        std::getline(std::cin, buffer);

        if (buffer == "takeoff")
        {
            try_offboard(m_drone->set_relative_position_ned(0, 0, -1.75))
        }
        if (buffer == "start")
        {
            m_drone->set_local_position_setpoint();
            sleep(5)
            try_offboard(m_drone->set_local_position_ned(0.25, 0, 0))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0.5, 0, 0))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0.75, 0, 0))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(1, 0, 0))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(1, 0, -0.25))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(1, 0, -0.5))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(1, 0, -0.75))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(1, 0, -1))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0.75, 0, -1))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0.5, 0, -1))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0.25, 0, -1))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0, 0, -1))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0, 0, -0.75))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0, 0, -0.5))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0, 0, -0.25))
            sleep(1)
            try_offboard(m_drone->set_local_position_ned(0, 0, 0))
        }
        if (buffer == "pause")
        {
            try_offboard(m_drone->offboard_hold())
        }
        if (buffer == "exit")
        {
            break;
        }

        suave_log << std::endl;
    }

    m_drone->offboard_wait_for_land();

    suave_log << "Stopping task" << std::endl;

    for (auto& task: m_task) {
        task->stop();
    }

    suave_log << "Exiting..." << std::endl;

}

void SuavePathPlanner::shutdown() {
    suave_log << "SuavePathPlanner::shutdown()" << std::endl;
    
    m_drone->offboard_wait_for_land();
    
    for (auto& task: m_task) {
        task->stop();
    }
}
