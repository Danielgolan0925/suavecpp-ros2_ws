//
// Created by patrick on 9/5/24.
//

#include "MaskingSubscriber.h"

#include "../controllers/ControllerMacros.h"

void MaskingSubscriber::set_enable(bool enable)
{
    m_enable = enable;
}

void MaskingSubscriber::callback(const Vector3Msg::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received x: %f, y: %f, z: %f", msg->x, msg->y, msg->z);

    if (m_enable)
    {
        constexpr float MAX_VELOCITY = 0.25;
        constexpr float MAX_DELTA_VELOCITY = 2.0*MAX_VELOCITY;
        const auto x = static_cast<float>(msg->x / 100);
        const auto y = static_cast<float>(msg->y / 100);
        const auto z = static_cast<float>(msg->z / 100);

        Velocity velocity
        {
            MAX_VELOCITY * x, // forward m/s
            0 * MAX_VELOCITY * z, // right m/s
            MAX_VELOCITY * -y, // down m/s
            0 // yawspeed deg/s
        };

        if (m_prevVelocity && std::abs(velocity.forward_m_s - m_prevVelocity->forward_m_s) > MAX_DELTA_VELOCITY)
        {
            suave_err << "Large difference in forward velocity: " << velocity.forward_m_s << " vs " << m_prevVelocity->forward_m_s << std::endl;
            this->shutdown();
            return;
        }
        // if (m_prevVelocity && std::abs(velocity.right_m_s - m_prevVelocity->right_m_s) > MAX_DELTA_VELOCITY)
        // {
        //     suave_err << "Large difference in right velocity: " << velocity.right_m_s << " vs " << m_prevVelocity->right_m_s << std::endl;
        //     this->shutdown();
        //     return;
        // }
        if (m_prevVelocity && std::abs(velocity.down_m_s - m_prevVelocity->down_m_s) > MAX_DELTA_VELOCITY)
        {
            suave_err << "Large difference in down velocity: " << velocity.down_m_s << " vs " << m_prevVelocity->down_m_s << std::endl;
            this->shutdown();
            return;
        }

        const auto& result = m_drone->offboard().set_velocity_body(velocity);
        if (result == Offboard::Result::Success)
        {
            m_prevVelocity = velocity;
        }
        else
        {
            suave_log << "Error setting velocity: " << result << "\nF: " << velocity.forward_m_s << "\nR: " << velocity.right_m_s << "\nD:" << velocity.down_m_s << std::endl;
            this->shutdown();
        }
    }
}

void MaskingSubscriber::shutdown()
{
    suave_err << "Shutting down MaskingSubscriber... resorting to position hold" << std::endl;
    m_enable = false;
    m_drone->offboard_hold();
}
