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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

std::atomic<bool> m_publish_telemetry{false};
std::thread m_telemetry_thread;

SuavePathPlanner::SuavePathPlanner() :
    ISuaveController()
{
    try {
        if (const auto& system = connectToPX4(m_mavsdk)) {
            m_drone = std::make_unique<Drone>(system);
        }
    }
    catch (...) { /*ignore*/ }

    // Initialize ROS node
    node = std::make_shared<rclcpp::Node>("suave_path_planner");

    // Initialize subscriber
    m_controller_output_subscriber = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/drone/controller_output", 10,
        std::bind(&SuavePathPlanner::controller_output_callback, this, std::placeholders::_1)
    );
}

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

void SuavePathPlanner::controller_output_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() == 8) {
        m_controller_output_values = msg->data;

        //double w0 = m_controller_output_values[0];
        //double wx = m_controller_output_values[1];
        //double wy = m_controller_output_values[2];
        //double wz = m_controller_output_values[3];
        //double v0 = m_controller_output_values[4];
        //double vx = m_controller_output_values[5];
        //double vy = m_controller_output_values[6];
        //double vz = m_controller_output_values[7];

        float wz = m_controller_output_values[3];
        float vx = m_controller_output_values[5];
        float vy = m_controller_output_values[6];
        float vz = m_controller_output_values[7];

        //constexpr float MAX_VELOCITY = 0.5;
        //constexpr float MAX_YAWSPEED = 20; // deg/s

        Velocity velocity {
            vy, // forward m/s
            -vx, //right m/s
            vz, //down m/s
            -wz // yawspeed deg/s
        };

        const auto& result = m_drone->offboard().set_velocity_body(velocity);
        if (result != Offboard::Result::Success) {
            suave_log << "Error setting velocity: " << result << "\nF: " << velocity.forward_m_s << "\nR: " << velocity.right_m_s << "\nD:" << velocity.down_m_s << std::endl;
        }
    } else {
        suave_log << "Unexpected controller output size: " << msg->data.size() << std::endl;
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

    // Create a MultiThreadedExecutor to spin the node
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread([executor]() { executor->spin(); }).detach();

    while (true) 
    {
        suave_log << "Input action: ";
        std::string buffer{};
        std::getline(std::cin, buffer);

        if (buffer == "takeoff")
        {
            //try_offboard(m_drone->set_relative_position_ned(0, 0, -1.75));
            try_offboard(m_drone->set_relative_position_ned(0, 0, -20));
            start_telemetry_publishing();
        }
        if (buffer == "start")
        {
            m_drone->set_local_position_setpoint();
            sleep(1);
            std::system("python3 /home/suave/Dev/suavecpp-ros2_ws/src/suave_controls/suave_controls/dq_controller.py &");
            // try_offboard(m_drone->set_local_position_ned(20, 0, 0))
            //Forward

        }
        if (buffer == "pause")
        {
            try_offboard(m_drone->offboard_hold());
            stop_telemetry_publishing();
        }
        if (buffer == "rtab"){
            suave_log << "Exporting RTAB-Map database......." << std::endl;

            // Modify the database path and output directory as needed
            std::string database_path = "~/.ros/rtabmap.db";
            std::string output_dir = "~/rtab_files";
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
    
    m_drone->offboard_wait_for_land();
    
    for (auto& task: m_task) {
        task->stop();
    }
}
