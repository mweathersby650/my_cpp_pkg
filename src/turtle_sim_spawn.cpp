#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/srv/get_active_turtles.hpp"
#include "my_robot_interfaces/msg/spawned_turtle.hpp"

class TurtleSpawnNode : public rclcpp::Node
{
public:
    TurtleSpawnNode() : Node("turtle_spawner"), mFrequencyToSpawnNewTurtle(1)
    {
        // create internal bits
        publisher_ = this->create_publisher<my_robot_interfaces::msg::SpawnedTurtle>(this->mTopic, 10);

        // read in parameter to say how often to spawn new turtle
        this->declare_parameter("frequency_to_spawn", 1.0);
        this->mFrequencyToSpawnNewTurtle = this->get_parameter("frequency_to_spawn").as_double();
        this->declare_parameter("max_turtles_to_spawn", 1);
        this->MAX_NUMBER_OF_TURTLES = this->get_parameter("max_turtles_to_spawn").as_int();

        // timer to spawn single turtle every N seconds
        mThread = std::thread(std::bind(&TurtleSpawnNode::spawnTurtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle Sim Spawn Node has been started with %d nodes", mFrequencyToSpawnNewTurtle);
    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::SpawnedTurtle>::SharedPtr publisher_;
    int mFrequencyToSpawnNewTurtle;
    std::thread mThread;
    int mTotalTurtlesCount = 0;
    const std::string mTopic = "created_turtles";
    int MAX_NUMBER_OF_TURTLES = 1;
    std::vector<std::string> activeTurtlesList;
    int createRandomInt(int a, int b)
    {
        if (a > b)
            return createRandomInt(b, a);
        if (a == b)
            return a;
        return a + (rand() % (b - a));
    }
    void getActiveTurtles()
    {
    }
    void spawnTurtle()
    {

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");


        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }
        auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
        while (1)
        {
            if (mTotalTurtlesCount >= MAX_NUMBER_OF_TURTLES)
            {
                RCLCPP_INFO(this->get_logger(), "Maxiumum number of turtles created");
                break;
            }

            // sleep for mFrequencyToSpawnNewTurtle seconds
            std::this_thread::sleep_for(std::chrono::seconds(mFrequencyToSpawnNewTurtle));

            // create a random position for the new turtle
            auto pose = generatePose();
            request->x = pose.x;
            request->y = pose.y;

            // spawn a new turtle with a random position
            std::string newTurtleName = send(client, request);
            // publish the results
            publishResults(pose, newTurtleName);

            mTotalTurtlesCount++;
            std::string message = "[" + std::to_string(mTotalTurtlesCount) + "] Wrote name [" + newTurtleName + "] and position (" + std::to_string(request->x) + "," + std::to_string(request->y) + ") to topic " + this->mTopic;
            RCLCPP_INFO(this->get_logger(), message.c_str());
        }
    }
    void publishResults(turtlesim::msg::Pose pose, std::string name)
    {
        auto msg = my_robot_interfaces::msg::SpawnedTurtle();
        msg.new_turtle_name = name;
        msg.pose.x = pose.x;
        msg.pose.y = pose.y;
        publisher_->publish(msg);
    }
    turtlesim::msg::Pose generatePose()
    {
        auto pose = turtlesim::msg::Pose();
        pose.x = createRandomInt(1, 9);
        pose.y = createRandomInt(1, 9);

        RCLCPP_INFO(this->get_logger(), "Publishing new turtle");
        return pose;
    }
    std::string send(std::shared_ptr<rclcpp::Client<turtlesim::srv::Spawn>> client, std::shared_ptr<turtlesim::srv::Spawn_Request> request)
    {
        auto future = client->async_send_request(request);
        std::string name;
        try
        {
            auto response = future.get();
            name = response->name;
        }
        catch (const std::exception &e)
        {

            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        RCLCPP_INFO(this->get_logger(), "Spawning new turtle");

        return name;
    }
    
};

int main(int argc, char **argv)
{
    // seeds the generator
    srand(time(0));
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}