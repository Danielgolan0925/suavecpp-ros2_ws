//
// Created by patrick on 9/5/24.
//

#include "SuaveMaskingPathPlanner.h"

#include "../common/SystemTask.h"
#include "../ros/RosNodeSpinner.h"
#include "../vio/CloudExporter.h"
#include "../vio/VIOBridge.h"
#include "ControllerMacros.h"
#include <atomic>
#include <thread>

std::atomic<bool> m_publish_telemetry{false};
std::thread m_telemetry_thread;

void SuavePathPlanner::start_telemetry_publishing() {
    m_publish_telemetry = true;
    m_telemetry_thread = std::thread([this]() {
        while (m_publish_telemetry) {
            m_drone->publish_telemetry();
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    });
}

void SuavePathPlanner::stop_telemetry_publishing() {
    m_publish_telemetry = false;
    if (m_telemetry_thread.joinable()) {
        m_telemetry_thread.join();
    }
}

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

    // Initialize ROS publisher
    m_drone->init_ros_publisher();
    m_drone->publish_telemetry();

    // Start offboard and arm
    try_action(m_drone->action().arm());
    try_offboard(m_drone->offboard_setpoint());
    try_offboard(m_drone->offboard().start());
    
    while (true) 
    {
        suave_log << "Input action: ";
        std::string buffer{};
        std::getline(std::cin, buffer);

        if (buffer == "takeoff")
        {
            try_offboard(m_drone->set_relative_position_ned(0, 0, -1.75));
            start_telemetry_publishing();
        }
        if (buffer == "start")
        {
            m_drone->set_local_position_setpoint();
            sleep(1);
            // try_offboard(m_drone->set_local_position_ned(20, 0, 0))
            // Forward
            double d = 5; // distance
            //double a = 0.5;
            //double n = 0.25;
            double b = 2; // distance
            for (double i = 0; i < d; i += 0.25) {
                try_offboard(m_drone->set_local_position_ned(i, 0, 0));
                for (int j = 0; j < 4; ++j) { 
                    std::string quaternion_str = m_drone->get_quaternion_string();
                    std::string ned_position_str = m_drone->get_ned_position_string();
                    std::string velocity_str = m_drone->get_velocity_string(); // Retrieve velocity string
                    suave_log << quaternion_str << " | " << ned_position_str << " | " << velocity_str << "\n"; // Print velocity
                    m_drone->publish_velocity(); // Publish velocity to ROS
                    std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
                }
            }
            
            // Right
            for (double i = 0; i < b; i += 0.25) {
                try_offboard(m_drone->set_local_position_ned(d, i, 0));
                for (int j = 0; j < 4; ++j) { 
                    std::string quaternion_str = m_drone->get_quaternion_string();
                    std::string ned_position_str = m_drone->get_ned_position_string();
                    std::string velocity_str = m_drone->get_velocity_string(); // Retrieve velocity string
                    suave_log << quaternion_str << " | " << ned_position_str << " | " << velocity_str << "\n"; // Print velocity
                    m_drone->publish_velocity(); // Publish velocity to ROS
                    std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
                }
            }

            // Backward
            for (double i = 0; i < d; i += 0.25) {
                try_offboard(m_drone->set_local_position_ned(d-i, b, 0));
                for (int j = 0; j < 4; ++j) { 
                    std::string quaternion_str = m_drone->get_quaternion_string();
                    std::string ned_position_str = m_drone->get_ned_position_string();
                    std::string velocity_str = m_drone->get_velocity_string(); // Retrieve velocity string
                    suave_log << quaternion_str << " | " << ned_position_str << " | " << velocity_str << "\n"; // Print velocity
                    m_drone->publish_velocity(); // Publish velocity to ROS
                    std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
                }
            }

            // Right
            for (double i = 0; i < b; i += 0.25) {
                try_offboard(m_drone->set_local_position_ned(0, b+i, 0));
                for (int j = 0; j < 4; ++j) { 
                    std::string quaternion_str = m_drone->get_quaternion_string();
                    std::string ned_position_str = m_drone->get_ned_position_string();
                    std::string velocity_str = m_drone->get_velocity_string(); // Retrieve velocity string
                    suave_log << quaternion_str << " | " << ned_position_str << " | " << velocity_str << "\n"; // Print velocity
                    m_drone->publish_velocity(); // Publish velocity to ROS
                    std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
                }
            }

            // Forward
            for (double i = 0; i < d; i += 0.25) {
                try_offboard(m_drone->set_local_position_ned(i, 2*b, 0));
                for (int j = 0; j < 4; ++j) { 
                    std::string quaternion_str = m_drone->get_quaternion_string();
                    std::string ned_position_str = m_drone->get_ned_position_string();
                    std::string velocity_str = m_drone->get_velocity_string(); // Retrieve velocity string
                    suave_log << quaternion_str << " | " << ned_position_str << " | " << velocity_str << "\n"; // Print velocity
                    m_drone->publish_velocity(); // Publish velocity to ROS
                    std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
                }
            }
        }
        if (buffer == "rtab"){
            suave_log << "Exporting RTAB-Map database......." << std::endl;

            // Modify the database path and output directory as needed
            std::string database_path = "~/.ros/rtabmap.db";
            std::string output_dir = "~/rtab_files";

            //std::string export_command = "rtabmap-export --output my_cloud --output_dir "+ output_dir +" "+database_path;
            //std::string export_command = "rtabmap-export --output " + filename.str() + " --output_dir ~/rtab_files ~/.ros/rtabmap.db";
            std::string exportcommand = "rtabmap-export --output $(date +cloud%Y-%m-%d_%H-%M-%S) --output_dir ~/rtab_files ~/.ros/rtabmap.db && rtabmap-export --decimation --noisefiltering --output $(date +cloud%Y-%m-%d_%H-%M-%S_big) --output_dir ~/rtab_files ~/.ros/rtabmap.db";

            int result = std::system(exportcommand.c_str());

            if (result == 0)
            {
                suave_log << "RTAB-Map export completed successfully." << std::endl;
            }
            else
            {
                suave_log << "Error exporting RTAB-Map data." << std::endl;
            }
        }
        if (buffer == "pause")
        {
            try_offboard(m_drone->offboard_hold());
            stop_telemetry_publishing();
        }
        if (buffer == "exit")
        {
            stop_telemetry_publishing();
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
    
    // suave_log << "Exporting RTAB-Map database due to Ctrl C......." << std::endl;

    //         // Modify the database path and output directory as needed
    //         std::string database_path = "~/.ros/rtabmap.db";
    //         std::string output_dir = "~/rtab_files";

    //         //std::string export_command = "rtabmap-export --output my_cloud --output_dir "+ output_dir +" "+database_path;
    //         //std::string export_command = "rtabmap-export --output " + filename.str() + " --output_dir ~/rtabfiles ~/.ros/rtabmap.db";
    //         std::string exportcommand = "rtabmap-export --output $(date +cloud%Y-%m-%d%H-%M-%S) --output_dir ~/rtab_files ~/.ros/rtabmap.db";

    //         int result = std::system(exportcommand.c_str());

    //         if (result == 0)
    //         {
    //             suave_log << "RTAB-Map export completed successfully." << std::endl;
    //         }
    //         else
    //         {
    //             suave_log << "Error exporting RTAB-Map data." << std::endl;
    //         }
        
    m_drone->offboard_wait_for_land();
    
    for (auto& task: m_task) {
        task->stop();
    }
}
