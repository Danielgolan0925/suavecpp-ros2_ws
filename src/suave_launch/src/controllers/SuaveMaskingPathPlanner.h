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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class SuavePathPlanner: public ISuaveController {
public:
    SuavePathPlanner();
    void start() override;
    void shutdown() override;

private:
    void start_telemetry_publishing();
    void stop_telemetry_publishing();
    void controller_output_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    using Velocity = Offboard::VelocityBodyYawspeed;

    bool m_end_controller{ false };
    std::unique_ptr<Drone> m_drone{ nullptr };
    std::vector<std::shared_ptr<ITask>> m_task{};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_controller_output_subscriber;
    std::shared_ptr<rclcpp::Node> node;
    std::vector<double> m_controller_output_values;
};

#endif //SUAVEMASKINGPATHPLANNER_H
