#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <termios.h>
#include <fcntl.h>
#include <cstdio>
#include <chrono>
using namespace std::chrono_literals;

bool isKeyPressed(int key) {
    struct termios oldt, newt;
    int oldf;
    char ch;
    bool keyPressed = false;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();
    if (ch == key) {
        keyPressed = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return keyPressed;
}

class KeyCheckNode : public rclcpp::Node {
public:
    KeyCheckNode() : Node("key_check_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("key_status", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&KeyCheckNode::checkKeyPress, this));
    }

private:
    void checkKeyPress() {
        auto message = std_msgs::msg::Bool();
        message.data = isKeyPressed('t');
        publisher_->publish(message);

        if (message.data) {
            RCLCPP_INFO(this->get_logger(), "Key 't' is pressed");
        } else {
            RCLCPP_INFO(this->get_logger(), "Key 't' is not pressed");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyCheckNode>());
    rclcpp::shutdown();
    return 0;
}

