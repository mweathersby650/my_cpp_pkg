 #include "rclcpp/rclcpp.hpp"
 #include "example_interfaces/msg/int64.hpp"
 #include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {

        server_ = this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",
            std::bind(&NumberCounterNode::handle_service, this, _1, _2)); 
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
        const std::string topicName = "number";
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(topicName, 10, std::bind(&NumberCounterNode::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "NumberCounter subscriber has been started.");
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

    }
 
private:

    int counter = 0;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    void callback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "number counter = %d", counter);

        counter += msg->data;
        RCLCPP_INFO(this->get_logger(), "New number received = %ld adding to counter %d", msg->data, counter);
        auto outMsg = example_interfaces::msg::Int64();
        outMsg.data = counter;
        publisher_->publish(outMsg);
    }
    void handle_service(const example_interfaces::srv::SetBool_Request::SharedPtr request,
                            const example_interfaces::srv::SetBool_Response::SharedPtr response )
    {

        response->message = "Counter remains unchanged at " + std::to_string(counter);
        if(request->data == true) {
            counter = 0;
            response->message = "Counter value set to " + std::to_string(counter);
        } 
        response->success = true;

        //RCLCPP_INFO(this->get_logger(), "data=%s  message=%s", response->success, response->message);
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