#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher() : Node("cmd_vel_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&CmdVelPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TwistStamped msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_link";

        msg.twist.linear.x = 0.2;   // constant velocity
        msg.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(),
                    "Publishing cmd_vel: v=%.2f w=%.2f",
                    msg.twist.linear.x,
                    msg.twist.angular.z);

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
