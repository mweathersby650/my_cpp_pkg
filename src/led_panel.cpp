#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include <sstream>

using std::placeholders::_1;

using std::placeholders::_2;
class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel_node")
    {
        // this->declare_parameter("led_status", rclcpp::PARAMETER_INTEGER_ARRAY);
        // rclcpp::Parameter int_array_param = this->get_parameter("led_status");
        this->declare_parameter<std::vector<int64_t>>("led_status", std::vector<int64_t>{0, 0, 0});
        led_status = this->get_parameter("led_status").as_integer_array();
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(&LedPanelNode::SetLedCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "led_panel Node has been started with value [%ld %ld %ld]", led_status[0], led_status[1], led_status[2]);
    }

private:
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    std::vector<int64_t> led_status;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr publisher_;

    void SetLedCallback(const my_robot_interfaces::srv::SetLed_Request::SharedPtr request,
                        const my_robot_interfaces::srv::SetLed_Response::SharedPtr response)
    {

        led_status[0] = request->led[0];
        led_status[1] = request->led[1];
        led_status[2] = request->led[2];
        std::ostringstream oss;
        oss << "setting led_panel to " << led_status[0] << "," << led_status[1] << "," << led_status[2];

        RCLCPP_INFO(this->get_logger(), "setting led_panel to: %ld %ld %ld", led_status[0], led_status[1], led_status[2]);

        auto msg = my_robot_interfaces::msg::LedStates();
        msg.led_state = {led_status[0], led_status[1], led_status[2]};
        msg.debug_message = oss.str();
        publisher_->publish(msg);
        response->exit = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}