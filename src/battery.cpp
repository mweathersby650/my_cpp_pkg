#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node 
{
public:
    BatteryNode() : Node("battery_node") 
    {
        thread1_ = std::thread(std::bind(&BatteryNode::publishBatteryEmpty,this));
        RCLCPP_INFO(this->get_logger(), "Battery Node has been started.");
    }
 
private:
    std::string battery_state = "full";  //empty or full
    std::string robot_name_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::thread thread1_;
    void publishBatteryEmpty()
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed_Request>();
        while(1) {

            std::this_thread::sleep_for(std::chrono::seconds(4));
            request->led[2] = 1;
            send(client,request);
            std::this_thread::sleep_for(std::chrono::seconds(6));
            request->led[2] = 0;
            send(client,request);
        }
    }
    void send(std::shared_ptr<rclcpp::Client<my_robot_interfaces::srv::SetLed>> client,std::shared_ptr<my_robot_interfaces::srv::SetLed_Request> request ) {
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Service return code is %d", response->exit);
        }
        catch (const std::exception &e)
        {

            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }


    }
    void publishBatteryFull()
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed_Request>();
        request->led[2] = 0;
        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Service return code is %d", response->exit);
        }
        catch (const std::exception &e)
        {

            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
    /*
    void publishBatteryFull()
    {
        auto msg = my_robot_interfaces::msg::LedStates();

        msg.led_state[0] = 0;
        msg.led_state[1] = 0;
        msg.led_state[2] = 0;
        msg.led_state = {0,0,0};
        msg.debug_message = "Battery is full";
        publisher_->publish(msg);
    }
    */
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}