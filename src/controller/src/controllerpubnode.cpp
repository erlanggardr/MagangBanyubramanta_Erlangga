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

#define X_CMD_MAX 250.0
#define X_CMD_MIN -250.0
#define Y_CMD_MAX 250.0
#define Y_CMD_MIN -250.0
#define YAW_CMD_MAX 180.0
#define YAW_CMD_MIN -180.0
#define DEPTH_CMD_MAX 10.0
#define DEPTH_CMD_MIN 0.0

class Publisher : public rclcpp::Node
{
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
	rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

	void callback_topic(const sensor_msgs::msg::Joy &msg)
	{
		auto cmd = interfaces::msg::Command();

		
		auto x_cmd = msg.axes[0] * X_CMD_MAX;  
		x_cmd = std::clamp(x_cmd, X_CMD_MIN, X_CMD_MAX);

		auto y_cmd = msg.axes[1] * Y_CMD_MAX;  
		y_cmd = std::clamp(y_cmd, Y_CMD_MIN, Y_CMD_MAX);

		auto yaw_cmd = msg.axes[3] * YAW_CMD_MAX;  
		yaw_cmd = std::clamp(yaw_cmd, YAW_CMD_MIN, YAW_CMD_MAX);

		auto depth_cmd = ((msg.axes[4] + 1) / 2) * DEPTH_CMD_MAX;  
		depth_cmd = std::clamp(depth_cmd, DEPTH_CMD_MIN, DEPTH_CMD_MAX);

		cmd.x = x_cmd;
		cmd.y = y_cmd;
		cmd.yaw = yaw_cmd;
		cmd.depth = depth_cmd;

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
