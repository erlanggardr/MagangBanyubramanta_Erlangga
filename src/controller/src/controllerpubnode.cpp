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

#define X_CMD_MAX 250.0f
#define X_CMD_MIN -250.0f
#define Y_CMD_MAX 250.0f
#define Y_CMD_MIN -250.0f
#define YAW_CMD_MAX 180.0f
#define YAW_CMD_MIN -180.0f
#define DEPTH_CMD_MAX 10.0f
#define DEPTH_CMD_MIN 0.0f

class Publisher : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

    float yaw_value = 0.0f;  
    float depth_value = 0.0f;  

    void callback_topic(const sensor_msgs::msg::Joy &msg)
    {
        auto cmd = interfaces::msg::Command();

        auto x_cmd = msg.axes[0] * -X_CMD_MAX;  
        x_cmd = std::clamp(x_cmd, X_CMD_MIN, X_CMD_MAX);

        auto y_cmd = msg.axes[1] * Y_CMD_MAX;
        y_cmd = std::clamp(y_cmd, Y_CMD_MIN, Y_CMD_MAX);

        if (msg.axes[3] != 0.0f) {
            yaw_value -= msg.axes[3] * 5.0f;  // Invert yaw axis and increment by a factor (e.g., 5.0)
            // Wrap yaw value around the range [-180, 180]
            if (yaw_value > YAW_CMD_MAX) yaw_value -= 360.0f;
            if (yaw_value < YAW_CMD_MIN) yaw_value += 360.0f;
        }

        // Invert depth and accumulate value if joystick input is not zero
        if (msg.axes[4] != 0.0f) {
            depth_value -= msg.axes[4] * 0.1f;  // Invert depth axis and adjust by a small value for gradual increase
            depth_value = std::clamp(depth_value, DEPTH_CMD_MIN, DEPTH_CMD_MAX);  // Keep within limits
        }

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
