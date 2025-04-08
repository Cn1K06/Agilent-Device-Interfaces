#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib> // For using system()
#include <algorithm> // For std::transform

class Agilent3320ANode : public rclcpp::Node {
public:
  Agilent3320ANode() : Node("agilent_3320a_node") {
    set_permissions(); // Set USBTMC permissions
    publisher_ = this->create_publisher<std_msgs::msg::String>("waveform_settings", 10);

    // First, learn the device's identity
    identity_ = send_command("*IDN?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    // Reset and clear status
    // reset_device();
    // clear_status();

    // Prompt user for waveform type
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

  void reset_device() {
    std::string response = send_command("*RST");
    RCLCPP_INFO(this->get_logger(), "Instrument reset");

    // Increase delay to 2 seconds
    usleep(2000000);

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

    // Check if the device is responsive
    response = send_command("*IDN?");
    if (response.empty() || response == "No response") {
      RCLCPP_ERROR(this->get_logger(), "Device not responding after status clear");
    } else {
      RCLCPP_INFO(this->get_logger(), "Device responsive after status clear: %s", response.c_str());
    }
  }

  void enable_output() {
    std::string response = send_command("OUTP ON");
    RCLCPP_INFO(this->get_logger(), "Output enabled.");
  }

  void disable_output() {
    std::string response = send_command("OUTP OFF");
    RCLCPP_INFO(this->get_logger(), "Output disabled. ");
  }

  void set_sine_wave() {
    double frequency = 1000.0; // 1kHz
    double amplitude = 5.0;    // 5Vpp
    double offset = 0.0;       // 0V offset
    std::string command = "APPL:SIN " + std::to_string(frequency) + "," +
                          std::to_string(amplitude) + "," + std::to_string(offset);
    std::string response = send_command(command);
    RCLCPP_INFO(this->get_logger(), "Sine Wave Settings");
  }

  void set_square_wave() {
    double frequency = 2000.0; // 2kHz
    double amplitude = 5.0;    // 5Vpp
    double offset = 0.0;       // 0V offset
    std::string command = "APPL:SQU " + std::to_string(frequency) + "," +
                          std::to_string(amplitude) + "," + std::to_string(offset);
    std::string response = send_command(command);
    RCLCPP_INFO(this->get_logger(), "Square Wave Settings");
  }

  void set_ramp_wave() {
    double frequency = 500.0; // 500Hz
    double amplitude = 5.0;    // 5Vpp
    double offset = 0.0;       // 0V offset
    std::string command = "APPL:RAMP " + std::to_string(frequency) + "," +
                          std::to_string(amplitude) + "," + std::to_string(offset);
    std::string response = send_command(command);
    RCLCPP_INFO(this->get_logger(), "Ramp Wave Settings");
  }

  void set_pulse_wave() {
    double frequency = 3000.0; // 3kHz
    double amplitude = 5.0;    // 5Vpp
    double offset = 0.0;       // 0V offset
    std::string command = "APPL:PULS " + std::to_string(frequency) + "," +
                          std::to_string(amplitude) + "," + std::to_string(offset);
    std::string response = send_command(command);
    RCLCPP_INFO(this->get_logger(), "Pulse Wave Settings");
  }

  void set_noise_wave() {
    double amplitude = 2.0;    // 2Vpp
    std::string command = "APPL:NOIS ," + std::to_string(amplitude);
    std::string response = send_command(command);
    RCLCPP_INFO(this->get_logger(), "Noise Wave Settings");
  }

  void configure_arbitrary_wave(double frequency = 1000.0, double amplitude = 1.0) {
    send_command("FUNC ARB");
    send_command("FREQ " + std::to_string(frequency));
    send_command("VOLT " + std::to_string(amplitude));

    // Example arbitrary waveform points
    std::vector<double> waveform_points = {0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5};

    // Convert points to string format for transmission
    std::string points_string;
    for (size_t i = 0; i < waveform_points.size(); ++i) {
      points_string += std::to_string(waveform_points[i]);
      if (i != waveform_points.size() - 1) {
        points_string += ", ";
      }
    }

    std::string response = send_command("DATA:ARB " + points_string);
    RCLCPP_INFO(this->get_logger(), "Arbitrary Wave Settings: '%s'", response.c_str());
  }

  void prompt_user() {
    std::string input;
    while (true) {
      RCLCPP_INFO(this->get_logger(), "Enter waveform type (SIN/SQU/RAMP/PULS/NOIS/ARB/STOP/EXIT):");
      std::getline(std::cin, input);

      // Convert input to uppercase
      std::transform(input.begin(), input.end(), input.begin(), ::toupper);

      if (input == "EXIT") {
        RCLCPP_INFO(this->get_logger(), "Exiting and shutting down node...");
        rclcpp::shutdown();
        break;
      } else if (input == "STOP") {
        RCLCPP_INFO(this->get_logger(), "Stopping waveform generation...");
        continue;
      } else {
        configure_waveform(input);
      }
    }
  }

  void configure_waveform(const std::string &waveform_type) {
    if (waveform_type == "SIN") {
      set_sine_wave();
    } else if (waveform_type == "SQU") {
      set_square_wave();
    } else if (waveform_type == "RAMP") {
      set_ramp_wave();
    } else if (waveform_type == "PULS") {
      set_pulse_wave();
    } else if (waveform_type == "NOIS") {
      set_noise_wave();
    } else if (waveform_type == "ARB") {
      configure_arbitrary_wave();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown waveform type: '%s'", waveform_type.c_str());
    }

    auto message = std_msgs::msg::String();
    message.data = "Waveform Settings: Type=" + waveform_type;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agilent3320ANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
