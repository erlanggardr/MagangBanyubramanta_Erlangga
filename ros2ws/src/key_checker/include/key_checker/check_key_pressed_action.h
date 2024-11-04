#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class CheckKeyPressedAction : public BT::SyncActionNode {
public:
    CheckKeyPressedAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), node_(std::make_shared<rclcpp::Node>("key_status_checker")) {
        subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
            "key_status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { key_pressed_ = msg->data; });
    }

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override {
        if (key_pressed_) {
            RCLCPP_INFO(node_->get_logger(), "Key 't' is pressed - Success");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Key 't' is not pressed - Failure");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    bool key_pressed_ = false;
};

