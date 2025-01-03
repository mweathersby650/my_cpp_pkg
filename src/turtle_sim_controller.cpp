#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/spawned_turtle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <signal.h>
#include "turtlesim/srv/kill.hpp"

class TurtleSimControllerNode : public rclcpp::Node
{
public:
    class Turtle
    {
    public:
        std::string name;
        turtlesim::msg::Pose pose;
        Turtle() {}
    };
    TurtleSimControllerNode() : Node("turtlesim_controller")
    {

        this->declare_parameter("linear_velocity", .5); // how fast to drive forward
        this->mLinearVelocity = this->get_parameter("linear_velocity").as_double();

        this->declare_parameter("angular_velocity", .5); // how fast to spin
        this->mAngularVelocity = this->get_parameter("angular_velocity").as_double();

        // listen to topic to find new turtles
        subscriber_ = this->create_subscription<my_robot_interfaces::msg::SpawnedTurtle>(this->CREATED_TURTLES_TOPIC, 10, std::bind(&TurtleSimControllerNode::callback, this, std::placeholders::_1));

        // listen to main turtle topic to find current position
        mMainTurtleCmd = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleSimControllerNode::mainTurtleCallback, this, std::placeholders::_1));
        mTimerToDriveTurtle = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&TurtleSimControllerNode::driveToTurtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle Sim Controller Node has been started");
    }

private:
    std::vector<Turtle *> turtles;
    double mLinearVelocity = .5;
    double mAngularVelocity = 10;
    rclcpp::TimerBase::SharedPtr mTimerToDriveTurtle;
    const std::string CREATED_TURTLES_TOPIC = "created_turtles";
    rclcpp::Subscription<my_robot_interfaces::msg::SpawnedTurtle>::SharedPtr subscriber_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr mMainTurtleCmd;
    geometry_msgs::msg::Twist currentTwist;

    turtlesim::msg::Pose currentPose;

    void mainTurtleCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {

        currentPose.x = msg->x;
        currentPose.y = msg->y;
        currentPose.theta = msg->theta;
    }
    void callback(const my_robot_interfaces::msg::SpawnedTurtle::SharedPtr msg)
    {
        std::string turtleName = msg->new_turtle_name;
        auto turtlePose = msg->pose;
        std::string message = "New turtle received: [name=" + turtleName + " (" + std::to_string(turtlePose.x) + "," + std::to_string(turtlePose.y) + ") ]";
        RCLCPP_INFO(this->get_logger(), message.c_str());
        Turtle *turtle = new Turtle();
        turtle->name = turtleName;
        turtle->pose = turtlePose;
        turtles.push_back(turtle);
    }

    double square(double base)
    {
        return pow(base, 2);
    }
    double calculateEuclideanDistance(int x, int y)
    {
        return sqrt(square(x - currentPose.x) + square(y - currentPose.y));
    }
    double calculateGoalToAngle(int x, int y)
    {
        return atan2(y - currentPose.y, x - currentPose.x);
    }

    // get main turtles current pose

    void drive(std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist, std::allocator<void>>> publisher, double distance, double angle)
    {
        // write the turtle1/cmd_vel
        geometry_msgs::msg::Twist twistMsg;
        twistMsg.linear.x = distance;
        twistMsg.angular.z = angle;
        publisher->publish(twistMsg);
    }
    void driveToTurtle()
    {
        double distanceErrorThreshold = 1; //.3;
        double angleErrorThreshold = .3;

        auto publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        if (turtles.empty() == true)
        {

            RCLCPP_DEBUG(this->get_logger(), "No turtles yet2");

            drive(publisher, 0, 0);
            return;
        }
        Turtle *turtle = turtles.front();

        // calculate the distance to the goal
        auto distanceToGoal = calculateEuclideanDistance(turtle->pose.x, turtle->pose.y);

        // calculate the angle to the new turtle
        auto angleToGoal = calculateGoalToAngle(turtle->pose.x, turtle->pose.y);

        // figure out how far off the mark the main turtle is relative to the newly spanwed one
        auto angleDifference = abs(angleToGoal - currentPose.theta);

        auto rotate = this->mAngularVelocity;

        // keep looping until we are pointed in the right direction
        if (angleDifference > angleErrorThreshold)
        {
            // rotate in the most efficient direction
            if (angleToGoal < currentPose.theta)
            {
                rotate = -mAngularVelocity;
            }
            RCLCPP_DEBUG(this->get_logger(), "rotating: current_angle = %f, angle_to_goal = %f, diff=%f", currentPose.theta, angleToGoal, angleDifference);
            drive(publisher, 0, rotate);
        }
        else if (distanceToGoal > distanceErrorThreshold)
        {
            RCLCPP_DEBUG(this->get_logger(), "driving: new distance to goal = %f with angle %f", distanceToGoal, angleDifference);
            drive(publisher, mLinearVelocity, 0);
        }
        else
        {
            deleteTargetTurtle(turtle);
            turtles.erase(turtles.begin());
        }
    }
    void deleteTargetTurtle(Turtle *turtle)
    {
        // arrived, clear the target turtle
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        auto request = std::make_shared<turtlesim::srv::Kill_Request>();
        request->name = turtle->name;
        client->async_send_request(request);

        std::string msg = "killed turtle: " + turtle->name;
        RCLCPP_INFO(this->get_logger(), msg.c_str());
    }
};

void signal_handler(int signum)
{
    rclcpp::shutdown();
}

void shutdownCallback()
{
    RCLCPP_INFO(rclcpp::get_logger("turtlesim_controller"), "Shutting down gracefully...");
    // Perform any necessary cleanup tasks here
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Install signal handler
    signal(SIGINT, signal_handler);
    // Register shutdown callback
    rclcpp::on_shutdown(shutdownCallback);

    auto node = std::make_shared<TurtleSimControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}