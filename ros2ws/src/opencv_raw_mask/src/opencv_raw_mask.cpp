#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class VideoMaskPublisherNode : public rclcpp::Node {
public:
    VideoMaskPublisherNode(const std::string& video_path)
        : Node("video_mask_publisher_node"), cap_(video_path) {
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video file: %s", video_path.c_str());
            return;
        }

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera", 10);
        mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);

        // frame 30 FPS (1000 ms / 30 = 33 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&VideoMaskPublisherNode::publish_frame_and_mask, this)
        );

        cv::namedWindow("HSV Adjustments");
        cv::createTrackbar("Hue Min 1", "HSV Adjustments", &hue_min1_, 180);
        cv::createTrackbar("Hue Max 1", "HSV Adjustments", &hue_max1_, 180);
        cv::createTrackbar("Hue Min 2", "HSV Adjustments", &hue_min2_, 180);
        cv::createTrackbar("Hue Max 2", "HSV Adjustments", &hue_max2_, 180);
        cv::createTrackbar("Sat Min", "HSV Adjustments", &sat_min_, 255);
        cv::createTrackbar("Sat Max", "HSV Adjustments", &sat_max_, 255);
        cv::createTrackbar("Val Min", "HSV Adjustments", &val_min_, 255);
        cv::createTrackbar("Val Max", "HSV Adjustments", &val_max_, 255);
    }

private:
    void publish_frame_and_mask() {
        cv::Mat frame;
        if (cap_.read(frame)) {
            auto orig_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            orig_msg->header.stamp = this->now();
            image_publisher_->publish(*orig_msg);

            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            cv::Mat lower_red_mask, upper_red_mask, red_mask;
            cv::inRange(hsv_frame, cv::Scalar(hue_min1_, sat_min_, val_min_), cv::Scalar(hue_max1_, sat_max_, val_max_), lower_red_mask);
            cv::inRange(hsv_frame, cv::Scalar(hue_min2_, sat_min_, val_min_), cv::Scalar(hue_max2_, sat_max_, val_max_), upper_red_mask);

            cv::bitwise_or(lower_red_mask, upper_red_mask, red_mask);

            auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", red_mask).toImageMsg();
            mask_msg->header.stamp = this->now();
            mask_publisher_->publish(*mask_msg);

            cv::imshow("Original Frame", frame);
            cv::imshow("Masked", red_mask);
            cv::waitKey(1);

            RCLCPP_INFO(this->get_logger(), "Published frame and mask at 30 FPS");
        } else {
            RCLCPP_WARN(this->get_logger(), "End of video reached.");
            rclcpp::shutdown(); 
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // default for red
    int hue_min1_ = 0, hue_max1_ = 30;
    int hue_min2_ = 150, hue_max2_ = 180;
    int sat_min_ = 100, sat_max_ = 255;
    int val_min_ = 100, val_max_ = 255;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Please provide the video file path as a command-line argument.");
        return 1;
    }

    auto node = std::make_shared<VideoMaskPublisherNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
