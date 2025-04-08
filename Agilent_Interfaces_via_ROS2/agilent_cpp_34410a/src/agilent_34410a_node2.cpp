#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib> // For using system()

class Agilent34410ANode : public rclcpp::Node {
public:
  Agilent34410ANode() : Node("agilent_34410a_node") {
    set_permissions(); // Set USBTMC permissions
    publisher_ = this->create_publisher<std_msgs::msg::String>("measurements", 10);

    // First, learn the device's identity
    identity_ = send_command("*IDN?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    // Reset the instrument
    reset_instrument();

    // Clear the device status
    clear_status();

    // Perform zero calibration
    zero_calibration();

    // Ask the user for the measurement type
    prompt_user();
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

  void reset_instrument() {
    std::string response = send_command("*RST");
    RCLCPP_INFO(this->get_logger(), "Instrument reset");
  }

  void clear_status() {
    std::string response = send_command("*CLS");
    RCLCPP_INFO(this->get_logger(), "Status cleared");
  }

  void zero_calibration() {
    std::string response = send_command("CAL:ZERO:ALL");
    RCLCPP_INFO(this->get_logger(), "Zero Calibration is done.");
  }

/*
  void reset_instrument() {
    std::string response = send_command("*RST");
    RCLCPP_INFO(this->get_logger(), "Instrument reset");

    // Increase delay to 2 seconds
    usleep(2000000);

    // Attempt to re-establish connection
    set_permissions();

    // Check if the device is responsive
    response = send_command("*IDN?");
    if (response.empty() || response == "No response") {
      RCLCPP_ERROR(this->get_logger(), "Device not responding after reset");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device responsive after reset: %s", response.c_str());
    }
  }

  void clear_status() {
    std::string response = send_command("*CLS");
    RCLCPP_INFO(this->get_logger(), "Status cleared");

    // Increase delay to 2 seconds
    usleep(2000000);

    // Attempt to re-establish connection
    set_permissions();

    // Check if the device is responsive
    response = send_command("*IDN?");
    if (response.empty() || response == "No response") {
      RCLCPP_ERROR(this->get_logger(), "Device not responding after status clear");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device responsive after status clear: %s", response.c_str());
    }
  }

*/

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
        break;
      } else {
        perform_measurement(input);
      }
    }
  }

  void perform_measurement(const std::string &measurement_type) {
    std::string command;
    std::string measurement_name;

    if (measurement_type == "DC") {
      command = "MEAS:VOLT:DC?";
      measurement_name = "DC Voltage";
    } else if (measurement_type == "AC") {
      command = "MEAS:VOLT:AC?";
      measurement_name = "AC Voltage";
    } else if (measurement_type == "CURR_DC") {
      command = "MEAS:CURR:DC?";
      measurement_name = "DC Current";
    } else if (measurement_type == "CURR_AC") {
      command = "MEAS:CURR:AC?";
      measurement_name = "AC Current";
    } else if (measurement_type == "RES") {
      command = "MEAS:RES?";
      measurement_name = "Resistance";
    } else if (measurement_type == "CAP") {
      command = "MEAS:CAP?";
      measurement_name = "Capacitance";
    } else if (measurement_type == "TEMP") {
      command = "MEAS:TEMP?";
      measurement_name = "Temperature";
    } else if (measurement_type == "FREQ") {
      command = "MEAS:FREQ?";
      measurement_name = "Frequency";
    } else if (measurement_type == "CALIBRATE") {
      command = "*CAL?";
      measurement_name = "Calibration";
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown measurement type: '%s'", measurement_type.c_str());
      return;
    }

    std::string result = send_command(command);
    auto message = std_msgs::msg::String();
    message.data = measurement_name + ": " + result;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
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
