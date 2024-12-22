 #include "rclcpp/rclcpp.hpp"
 #include "my_robot_interfaces/msg/hardware_status.hpp"


class HardwareStatusPublisher : public rclcpp::Node
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher")
    {
        const std::string topicName = "hardware_status";
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(topicName, 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HardwareStatusPublisher::publishHardwareStatus, this));

        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started.");
    }
 
private:
    void publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 57;
        msg.debug_message = "Motors are too hot!";
        msg.are_motors_ready = false;
        publisher_->publish(msg);
    }
    std::string robot_name_;
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}