//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingController.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"
#include "ControllerMacros.h"
#include <thread>
#include <chrono>

void SuaveMaskingController::start() {
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

    // The block below to enables CPU logging
    auto cpu_logger = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "source ~/Dev/suavecpp-ros2_ws/install/setup.bash",
            "ros2 run suave_controls cpu_logger"
        }
    );
    m_task.push_back(cpu_logger);
    cpu_logger->start_in_thread();

    // Create ROS spinner and add vio bridge and cloud exporter node
    auto vio_spinner = std::make_shared<RosNodeSpinner>();
    m_task.push_back(vio_spinner);

    const auto vio_bridge_node = std::make_shared<VIOBridge>(m_drone->system(), m_drone->initial_heading_rad());
    vio_spinner->add_node(vio_bridge_node);

    const auto cloud_exporter_node = std::make_shared<CloudExporter>();
    vio_spinner->add_node(cloud_exporter_node);

    auto masking_spinner = std::make_shared<RosNodeSpinner>();
    m_task.push_back(masking_spinner);

    m_masking_subscriber = std::make_shared<MaskingSubscriber>(m_drone.get());
    masking_spinner->add_node(m_masking_subscriber);

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

    auto masking_pid_task = std::make_shared<SystemTask>(
        std::vector<std::string>{
            "source /opt/ros/humble/setup.bash",
            "source ~/Dev/suavecpp-ros2_ws/install/setup.bash",
            "ros2 run suave_controls masking_pid_publisher"
        }
    );

    auto export_task = SystemTask
            {
                std::vector<std::string>{
                    "source /opt/ros/humble/setup.bash",
                    "source ~/Dev/suavecpp-ros2_ws/install/setup.bash",
                    "ros2 service call /exportXYZ std_srvs/srv/Empty",
                    "ros2 service call /exportCPU std_srvs/srv/Empty"
                }
            };

    // Start offboard and arm
    try_action(m_drone->action().arm())
    try_offboard(m_drone->offboard_setpoint())
    try_offboard(m_drone->offboard().start())

    // Start a thread for publishing velocity
    std::thread velocity_publisher_thread([this]() {
        while (!m_end_controller) { 
            m_drone->publish_velocity();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }
    });

    // Detach the thread so it runs independently
    velocity_publisher_thread.detach();

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
            m_masking_subscriber->enable();
            masking_spinner->start_in_thread();
            masking_pid_task->start_in_thread();
        }
        if (buffer == "rtab"){
            suave_log << "Exporting RTAB-Map database......." << std::endl;

            // Modify the database path and output directory as needed
            std::string database_path = "~/.ros/rtabmap.db";
            std::string output_dir = "~/rtab_files";

            //std::string export_command = "rtabmap-export --output my_cloud --output_dir "+ output_dir +" "+database_path;
            //std::string export_command = "rtabmap-export --output " + filename.str() + " --output_dir ~/rtab_files ~/.ros/rtabmap.db";
            std::string export_command = "rtabmap-export --output $(date +cloud%Y-%m-%d_%H-%M-%S) --output_dir ~/rtab_files ~/.ros/rtabmap.db";

            int result = std::system(export_command.c_str());

            if (result == 0)
            {
                suave_log << "RTAB-Map export completed successfully." << std::endl;
            }
            else
            {
                suave_log << "Error exporting RTAB-Map data." << std::endl;
            }
        }
        if (buffer == "stop")
        {
            m_masking_subscriber->disable();
            masking_pid_task->stop();
            masking_spinner->stop();
            try_offboard(m_drone->offboard_hold())
        }
        if (buffer == "maskon")
        {
            masking_pid_task->start_in_thread();
        }
        if (buffer == "maskoff")
        {
            masking_pid_task->stop();
        }
        if (buffer == "exit")
        {
            break;
        }
        if (buffer == "export")
        {
            // export_task.start_in_thread();
            suave_log << "Exporting CPU Data" << std::endl;
            system("ros2 service call /exportCPU std_srvs/srv/Empty");
        }

        suave_log << std::endl;
    }

    export_task.start_in_thread();

    m_drone->offboard_wait_for_land();

    suave_log << "Stopping task" << std::endl;

    for (auto& task: m_task) {
        task->stop();
    }

    suave_log << "Exiting..." << std::endl;

}

void SuaveMaskingController::shutdown() {
    suave_log << "SuaveMaskingController::shutdown()" << std::endl;

    // Signal the velocity publisher thread to stop
    m_end_controller = true;

    if (m_masking_subscriber)
    {
        m_masking_subscriber->disable();
    }

    m_drone->offboard_wait_for_land();
    
    for (auto& task: m_task) {
        task->stop();
    }
}
