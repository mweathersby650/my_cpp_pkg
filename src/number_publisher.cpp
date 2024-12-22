 #include "rclcpp/rclcpp.hpp"
 #include "example_interfaces/msg/int64.hpp"


class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        const std::string topicName = "number";
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>(topicName, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::publishNews, this));

        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");
    }
 
private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = 64;
        publisher_->publish(msg);
    }
    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}