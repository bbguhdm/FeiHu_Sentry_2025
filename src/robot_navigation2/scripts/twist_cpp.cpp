#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <serial/serial.h>

class OdomSubscriber : public rclcpp::Node
{
public:
    OdomSubscriber()
    : Node("odom_subscriber")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomSubscriber::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("linear_speed", 10);
        serial_port_ = std::make_shared<serial::Serial>("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
    }

private:
    void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        auto linear_speed = msg->twist.twist.linear.x;
        auto angular_speed = msg->twist.twist.angular.z;
        auto z_speed = msg->twist.twist.angular.z;
        RCLCPP_INFO(this->get_logger(), "Linear speed: %f m/s, Angular speed: %f rad/s, z speed: %f", linear_speed, angular_speed, z_speed);
        
        std_msgs::msg::Float64 linear_speed_msg;
        linear_speed_msg.data = linear_speed;
        publisher_->publish(linear_speed_msg);
        
        if (serial_port_->isOpen())
        {
            std::string data_to_send = std::to_string(linear_speed) + "\n";
            serial_port_->write(data_to_send);
        }
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    std::shared_ptr<serial::Serial> serial_port_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    rclcpp::shutdown();
    return 0;
}