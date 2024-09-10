//
// Created by patrick on 9/5/24.
//

#include "MaskingSubscriber.h"

void MaskingSubscriber::callback(const Vector3Msg::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received x: %f, y: %f, z: %f", msg->x, msg->y, msg->z);

    // do something with m_drone here

}
