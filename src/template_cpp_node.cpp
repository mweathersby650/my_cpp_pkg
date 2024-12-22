 #include "rclcpp/rclcpp.hpp"

class AddTwoIntsServerNode : public rclcpp::Node // MODIFY NAME
{
public:
    AddTwoIntsServerNode() : Node("node_name") // MODIFY NAME
    {
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}