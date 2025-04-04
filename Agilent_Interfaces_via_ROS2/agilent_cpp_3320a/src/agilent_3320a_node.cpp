#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib> // For using system()

class Agilent3320ANode : public rclcpp::Node {
public:
  Agilent3320ANode() : Node("agilent_3320a_node") {
    set_permissions(); // Set USBTMC permissions
    publisher_ = this->create_publisher<std_msgs::msg::String>("waveform_settings", 10);

    // First, learn the device's identity
    identity_ = send_command("*IDN?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&Agilent3320ANode::timer_callback, this));
  }

private:
  const std::string usbtmc_device = "/dev/usbtmc1"; // Device file path
  std::string identity_; // Device identity storage

  void set_permissions() {
    std::string command = "sudo chmod 666 " + usbtmc_device;
    int ret = system(command.c_str());
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "Permissions set for %s", usbtmc_device.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set permissions for %s", usbtmc_device.c_str());
    }
  }

  std::string send_command(const std::string &command) {
    std::ofstream device_out(usbtmc_device, std::ios::out | std::ios::trunc);
    if (!device_out.is_open()) {
      return "Error: Cannot open USBTMC device for writing";
    }
    device_out << command << std::endl;
    device_out.flush();
    device_out.close();

    usleep(500000); // 500ms wait (for the device to respond)

    std::ifstream device_in(usbtmc_device, std::ios::in);
    if (!device_in.is_open()) {
      return "Error: Cannot open USBTMC device for reading";
    }

    std::string response;
    std::getline(device_in, response);
    device_in.close();

    return response.empty() ? "No response" : response;
  }

  void timer_callback() {
    // Set a sine wave with specific frequency, amplitude, and offset
    double frequency = 1000.0; // 1 kHz
    double amplitude = 5.0;    // 5Vpp
    double offset = 0.0;       // 0V offset

    std::string command = "APPL:SIN " + std::to_string(frequency) + "," +
                          std::to_string(amplitude) + "," + std::to_string(offset);
    std::string response = send_command(command);

    auto message = std_msgs::msg::String();
    message.data = "Sine Wave Settings: Frequency=" + std::to_string(frequency) +
                   "Hz, Amplitude=" + std::to_string(amplitude) +
                   "Vpp, Offset=" + std::to_string(offset) + "V";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agilent3320ANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
