#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib> // For using system()
#include <algorithm> // For using std::transform
#include <thread> // For using std::thread
#include <atomic> // For using std::atomic
#include <map> // For using std::map

class Agilent34410ANode : public rclcpp::Node {
public:
  Agilent34410ANode() : Node("agilent_34410a_node"), keep_measuring_(false) {
    set_permissions(); // Set USBTMC permissions
    publisher_ = this->create_publisher<std_msgs::msg::String>("measurements", 10);

    // First, learn the device's identity
    identity_ = send_command("*IDN?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    // Reset the instrument
    // reset_instrument();

    // Clear the device status
    // clear_status();

    // Ask the user for the measurement type
    prompt_user();
  }

private:
  const std::string usbtmc_device = "/dev/usbtmc1"; // Device file path
  std::string identity_; // Device identity storage
  std::atomic<bool> keep_measuring_;
  std::map<std::string, std::string> measurement_commands_ = {
      {"DC", "MEAS:VOLT:DC?"},
      {"AC", "MEAS:VOLT:AC?"},
      {"CURR_DC", "MEAS:CURR:DC?"},
      {"CURR_AC", "MEAS:CURR:AC?"},
      {"RES", "MEAS:RES?"},
      {"CAP", "MEAS:CAP?"},
      {"TEMP", "MEAS:TEMP?"},
      {"FREQ", "MEAS:FREQ?"}
  };

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

  void reset_instrument() {
    std::string response = send_command("*RST");
    RCLCPP_INFO(this->get_logger(), "Instrument reset");
  }

  void clear_status() {
    std::string response = send_command("*CLS");
    RCLCPP_INFO(this->get_logger(), "Status cleared");
  }

  void prompt_user() {
    std::string input;
    while (true) {
      RCLCPP_INFO(this->get_logger(), "Enter measurement type (DC/AC/CURR_DC/CURR_AC/RES/CAP/TEMP/FREQ/CALIBRATE/STOP/EXIT):");
      std::getline(std::cin, input);

      // Convert input to uppercase
      std::transform(input.begin(), input.end(), input.begin(), ::toupper);

      if (input == "EXIT") {
        RCLCPP_INFO(this->get_logger(), "Exiting and shutting down node...");
        rclcpp::shutdown();
        break;
      } else if (input == "STOP") {
        RCLCPP_INFO(this->get_logger(), "Stopping measurements...");
        keep_measuring_ = false;
      } else {
        perform_measurement(input);
      }
    }
  }

  void perform_measurement(const std::string &measurement_type) {
    if (measurement_commands_.find(measurement_type) != measurement_commands_.end()) {
      // Start continuous measurement in a separate thread
      keep_measuring_ = true;
      std::thread measurement_thread(&Agilent34410ANode::measure_continuous, this, measurement_type);
      measurement_thread.detach();
    } else if (measurement_type == "CALIBRATE") {
      std::string result = send_command("*CAL?");
      auto message = std_msgs::msg::String();
      message.data = "Calibration: " + result;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown measurement type: '%s'", measurement_type.c_str());
    }
  }

  void measure_continuous(const std::string &measurement_type) {
    while (keep_measuring_) {
      std::string command = measurement_commands_[measurement_type];
      std::string result = send_command(command);
      RCLCPP_INFO(this->get_logger(), "%s: %s", measurement_type.c_str(), result.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agilent34410ANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
