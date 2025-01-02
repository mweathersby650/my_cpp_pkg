#include "rclcpp/rclcpp.hpp"
// #include "turtlesim/turtlesim_node.hpp"

class TurtleSimWindowNode : public rclcpp::Node
{
public:
    TurtleSimWindowNode() : Node("turtle_sim_window")
    {
        // MATT - Don't know how to createa a turtlesim object in code, as such I am relyingon a launch file
        // start turtle simulation node
        // also according to google AI, turtlesim::Node class does not exist in ROS2 turtle
        // auto node = std::make_shared<turtlesim::Node>("add_two_ints_client_no_oop"); // MODIFY NAME
        // start a service waiting for new spawn commands
        RCLCPP_INFO(this->get_logger(), "Turtle Sim Window Node has been started...");
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSimWindowNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}