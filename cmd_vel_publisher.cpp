#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdvelPublisher : public rclcpp::Node
{
public:
    CmdvelPublisher() : Node("cmd_vel_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&CmdvelPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        msg.linear.x = 0.2;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdvelPublisher>());
    rclcpp::shutdown();
    return 0;
}