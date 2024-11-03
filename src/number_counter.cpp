 #include "rclcpp/rclcpp.hpp"
 #include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        const std::string topicName = "number";
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(topicName, 10, std::bind(&NumberCounterNode::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "NumberCounter subscriber has been started.");
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

    }
 
private:

    int counter = 0;
    void callback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "number counter = %d", counter);

        if (msg->data != counter) {
            counter += msg->data;
            RCLCPP_INFO(this->get_logger(), "New number received = %ld adding to counter %d", msg->data, counter);
            auto outMsg = example_interfaces::msg::Int64();
            outMsg.data = counter;
            publisher_->publish(outMsg);
        }
    }
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}