#include <rclcpp/rclcpp.hpp>
#include <interfaces/msg/command.hpp>
#include <asio.hpp>
#include <sstream>
#include <cmath>

class CmdVelToSerial : public rclcpp::Node {
public:
    CmdVelToSerial() : Node("cmd_vel_to_serial") {
        // Subscription ke topic cmd_vel
        subscription_ = this->create_subscription<interfaces::msg::Command>(
            "cmd_vel", 10, std::bind(&CmdVelToSerial::cmdVelCallback, this, std::placeholders::_1));

        // Inisialisasi serial port
        try {
            serial_.open("/dev/ttyACM0");
            serial_.set_option(asio::serial_port_base::baud_rate(115200));
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }
    }

    ~CmdVelToSerial() {
        if (serial_.is_open()) {
            serial_.close();
        }
    }

private:
    void cmdVelCallback(const interfaces::msg::Command::SharedPtr msg) {
        if (!serial_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            return;
        }

        // Mapping x, y, depth ke nilai PWM [0, 8]
        int pwm_x = mapValue(msg->x, -255, 255, 0, 8); // Kanan/kiri
        int pwm_y = mapValue(msg->y, -255, 255, 0, 8); // Maju/mundur
        double pwm_depth = mapValue(msg->depth, 0, 10, 0, 8); // Vertikal (0–4 turun, 4.1–8 naik)

        // Logika vertikal untuk turun/naik
        int vertical_pwm = 0;
        if (pwm_depth <= 4.0) {
            vertical_pwm = static_cast<int>(pwm_depth); // Skala 0-4 untuk turun
        } else {
            vertical_pwm = static_cast<int>(pwm_depth); // Skala 4.1-8 untuk naik
        }

        // Format data untuk dikirim
        std::ostringstream oss;
        oss << pwm_x << "," << pwm_y << "," << vertical_pwm << "\n";

        try {
            asio::write(serial_, asio::buffer(oss.str()));
            RCLCPP_INFO(this->get_logger(), "Sent to serial: %s", oss.str().c_str());
        } catch (std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send to serial: %s", e.what());
        }
    }

    // Fungsi untuk mapping nilai input ke rentang output
    double mapValue(double value, double in_min, double in_max, double out_min, double out_max) {
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    rclcpp::Subscription<interfaces::msg::Command>::SharedPtr subscription_;
    asio::io_context io_context_;
    asio::serial_port serial_{io_context_};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToSerial>());
    rclcpp::shutdown();
    return 0;
}

