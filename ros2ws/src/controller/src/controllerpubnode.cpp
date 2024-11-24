#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>  

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/command.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define X_CMD_MAX 250
#define X_CMD_MIN -250
#define Y_CMD_MAX 250
#define Y_CMD_MIN -250
#define YAW_CMD_MAX 180
#define YAW_CMD_MIN -180
#define DEPTH_CMD_MAX 10
#define DEPTH_CMD_MIN 0

class Publisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

    // Change internal state variables to integers
    int yaw_value = 0;  
    int depth_value = 0;  

    void callback_topic(const sensor_msgs::msg::Joy &msg)
    {
        auto cmd = interfaces::msg::Command();

        // Compute x_cmd as an integer
        int x_cmd = static_cast<int>(msg.axes[0] * -X_CMD_MAX);  
        x_cmd = std::clamp(x_cmd, X_CMD_MIN, X_CMD_MAX);

        // Compute y_cmd as an integer
        int y_cmd = static_cast<int>(msg.axes[1] * Y_CMD_MAX);
        y_cmd = std::clamp(y_cmd, Y_CMD_MIN, Y_CMD_MAX);

        // Update yaw_value as an integer
        if (msg.axes[3] != 0.0f) {
            // Multiply by an integer factor (e.g., 5) and cast to int
            yaw_value -= static_cast<int>(msg.axes[3] * 5.0f);  
            // Wrap yaw value around the range [-180, 180]
            if (yaw_value > YAW_CMD_MAX) yaw_value -= 360;
            if (yaw_value < YAW_CMD_MIN) yaw_value += 360;
        }

        // Update depth_value as an integer
        if (msg.axes[4] != 0.0f) {
            // Multiply by an integer factor (e.g., 1) for gradual change
            depth_value -= static_cast<int>(msg.axes[4] * 1.0f);  
            depth_value = std::clamp(depth_value, DEPTH_CMD_MIN, DEPTH_CMD_MAX);  // Keep within limits
        }

        // Assign integer values to the command message
        cmd.x = x_cmd;
        cmd.y = y_cmd;
        cmd.yaw = yaw_value;
        cmd.depth = depth_value;

        pub->publish(cmd);
    }

public:
    Publisher() : Node("node")
    {
        sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&Publisher::callback_topic, this, _1));

        pub = this->create_publisher<interfaces::msg::Command>("cmd_vel", 10);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}
/*
DEFINE CONSTANTS:
    X_CMD_MAX = 250, X_CMD_MIN = -250
    Y_CMD_MAX = 250, Y_CMD_MIN = -250
    YAW_CMD_MAX = 180, YAW_CMD_MIN = -180
    DEPTH_CMD_MAX = 10, DEPTH_CMD_MIN = 0

CLASS Publisher (inherits rclcpp::Node):
    VARIABLES:
        sub: Subscription to "joy" topic
        pub: Publisher to "cmd_vel" topic
        yaw_value: Integer, tracks yaw angle (default 0)
        depth_value: Integer, tracks depth (default 0)

    FUNCTION callback_topic(msg):
        x_cmd = Convert msg.axes[0] to integer scaled by -X_CMD_MAX
        Clamp x_cmd to range [X_CMD_MIN, X_CMD_MAX]

        y_cmd = Convert msg.axes[1] to integer scaled by Y_CMD_MAX
        Clamp y_cmd to range [Y_CMD_MIN, Y_CMD_MAX]

        IF msg.axes[3] is not zero:
            yaw_value -= Integer value of (msg.axes[3] * 5)
            Wrap yaw_value within range [-180, 180]

        IF msg.axes[4] is not zero:
            depth_value -= Integer value of (msg.axes[4] * 1)
            Clamp depth_value to range [DEPTH_CMD_MIN, DEPTH_CMD_MAX]

        CREATE command message:
            cmd.x = x_cmd
            cmd.y = y_cmd
            cmd.yaw = yaw_value
            cmd.depth = depth_value

        Publish command message via pub

    CONSTRUCTOR:
        Initialize ROS2 node
        Create subscription to "joy" topic
        Create publisher for "cmd_vel" topic

FUNCTION main():
    Initialize ROS2
    Create Publisher node instance
    Spin node (process callbacks)
    Shutdown ROS2
*/

/*
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>  

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/command.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#define X_CMD_MAX 250
#define X_CMD_MIN -250
#define Y_CMD_MAX 250
#define Y_CMD_MIN -250
#define YAW_CMD_MAX 180
#define YAW_CMD_MIN -180
#define DEPTH_CMD_MAX 10
#define DEPTH_CMD_MIN 0

class Publisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

    // Change internal state variables to integers
    int yaw_value = 0;  
    int depth_value = 0;  

    void callback_topic(const sensor_msgs::msg::Joy &msg)
    {
        auto cmd = interfaces::msg::Command();

        // Compute x_cmd as an integer
        int x_cmd = static_cast<int>(msg.axes[0] * -X_CMD_MAX);  
        x_cmd = std::clamp(x_cmd, X_CMD_MIN, X_CMD_MAX);

        // Compute y_cmd as an integer
        int y_cmd = static_cast<int>(msg.axes[1] * Y_CMD_MAX);
        y_cmd = std::clamp(y_cmd, Y_CMD_MIN, Y_CMD_MAX);

        // Update yaw_value as an integer
        if (msg.axes[3] != 0.0f) {
            // Multiply by an integer factor (e.g., 5) and cast to int
            yaw_value -= static_cast<int>(msg.axes[3] * 5.0f);  
            // Wrap yaw value around the range [-180, 180]
            if (yaw_value > YAW_CMD_MAX) yaw_value -= 360;
            if (yaw_value < YAW_CMD_MIN) yaw_value += 360;
        }

        // Update depth_value as an integer
        if (msg.axes[4] != 0.0f) {
            // Multiply by an integer factor (e.g., 1) for gradual change
            depth_value -= static_cast<int>(msg.axes[4] * 1.0f);  
            depth_value = std::clamp(depth_value, DEPTH_CMD_MIN, DEPTH_CMD_MAX);  // Keep within limits
        }

        // Assign integer values to the command message
        cmd.x = x_cmd;
        cmd.y = y_cmd;
        cmd.yaw = yaw_value;
        cmd.depth = depth_value;

        pub->publish(cmd);
    }

public:
    Publisher() : Node("node")
    {
        sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&Publisher::callback_topic, this, _1));

        pub = this->create_publisher<interfaces::msg::Command>("cmd_vel", 10);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}

*/
