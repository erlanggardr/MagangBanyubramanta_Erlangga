#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <interfaces/msg/object_command.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "object_detection/yolov5.h"
#include "object_detection/constants.h"

#include <memory>
#include <vector>

class ObjectDetectionNode : public rclcpp::Node
{
public:
    ObjectDetectionNode()
    : Node("object_detection_node")
    {
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&ObjectDetectionNode::image_callback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/objects_box", 10);
        objects_publisher_ = this->create_publisher<interfaces::msg::ObjectCommand>("/objects", 10);

        Config config = {0.4f, 0.4f, 0.4f, 640, 640, "src/object_detection/models/best.onnx"};
        yolomodel_ = std::make_shared<YOLOV5>(config);

        RCLCPP_INFO(this->get_logger(), "Object Detection Node Initialized");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // ros image to opencv
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            yolomodel_->detect(frame);

            // publish with bounding box
            sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            image_publisher_->publish(*out_msg);

            // publish detected objek
            std::vector<Detection> detections = yolomodel_->get_detections();
            for (const auto &det : detections) {
                if (det.class_id < 0 || det.class_id >= static_cast<int>(coconame_size)) {
                    continue; 
                }

                interfaces::msg::ObjectCommand obj_msg;
                obj_msg.name = coconame[det.class_id];
                obj_msg.x = det.box.x;
                obj_msg.y = det.box.y;
                obj_msg.width = det.box.width;
                obj_msg.height = det.box.height;
                obj_msg.confidence = det.confidence;
                objects_publisher_->publish(obj_msg);
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during detection: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<interfaces::msg::ObjectCommand>::SharedPtr objects_publisher_;

    std::shared_ptr<YOLOV5> yolomodel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
