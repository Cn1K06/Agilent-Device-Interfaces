#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib> // For using system()
#include <algorithm> // For std::transform
#include <chrono>
#include <sstream>

class Agilent3320ANode : public rclcpp::Node {
public:
  Agilent3320ANode() : Node("agilent_3320a_node") {
    set_permissions(); // Set USBTMC permissions
    publisher_ = this->create_publisher<std_msgs::msg::String>("waveform_settings", 10);

    // First, learn the device's identity
    identity_ = send_command("*IDN?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    // Reset and clear status
    reset_device();
    clear_status();

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

  std::string send_command(const std::string &command, bool read_response = true, int timeout_ms = 1000) {
    std::ofstream device_out(usbtmc_device, std::ios::out | std::ios::trunc);
    if (!device_out.is_open()) {
      return "Error: Cannot open USBTMC device for writing";
    }
    device_out << command << std::endl;
    device_out.flush();
    device_out.close();

    if (!read_response) {
      return "Command Sent";
    }

    auto start = std::chrono::steady_clock::now();
    std::ifstream device_in;
    std::string response;
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
      device_in.open(usbtmc_device, std::ios::in);
      if (device_in.is_open()) {
        std::getline(device_in, response);
        device_in.close();
        if (!response.empty()) {
          return response;
        }
      }
      usleep(1000000); // 1s wait
    }
    return "Timeout";
  }

  void reset_device() {
    send_command("*RST", false);
    usleep(2000000);
    RCLCPP_INFO(this->get_logger(), "Instrument reset.");
  }

  void clear_status() {
    send_command("*CLS", false);
    usleep(2000000);
    RCLCPP_INFO(this->get_logger(), "Status clear.");
  }

  void wait_for_operation_complete() {
    std::string response = send_command("*OPC?");
    RCLCPP_INFO(this->get_logger(), "Operation Complete: %s", response.c_str());
  }

  void check_device_status() {
    std::string response = send_command("SYST:ERR?");
    RCLCPP_INFO(this->get_logger(), "Device Status: %s", response.c_str());
  }

  void enable_output() {
    send_command("OUTP ON", false);
    RCLCPP_INFO(this->get_logger(), "Output enabled.");
  }

  void disable_output() {
    send_command("OUTP OFF", false);
    RCLCPP_INFO(this->get_logger(), "Output disabled.");
  }

  void set_sine_wave(double freq, double amp) {
    std::ostringstream command;
    command << "FUNC SIN";
    send_command(command.str(), false);
    send_command("FREQ " + std::to_string(freq), false);
    send_command("VOLT " + std::to_string(amp), false);
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
  }

  void set_square_wave(double freq, double amp, double duty_cycle) {
    std::ostringstream command;
    command << "FUNC SQU";
    send_command(command.str(), false);
    send_command("FREQ " + std::to_string(freq), false);
    send_command("VOLT " + std::to_string(amp), false);
    send_command("PWID " + std::to_string(duty_cycle), false);  // Set duty cycle
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
  }

  void set_ramp_wave(double freq, double amp, double symmetry) {
    std::ostringstream command;
    command << "FUNC RAMP";
    send_command(command.str(), false);
    send_command("FREQ " + std::to_string(freq), false);
    send_command("VOLT " + std::to_string(amp), false);
    send_command("SYMM " + std::to_string(symmetry), false);  // Set symmetry
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
  }

  void set_pulse_wave(double freq, double amp, double pulse_width, double edge_time) {
    std::ostringstream command;
    command << "FUNC PULS";
    send_command(command.str(), false);
    send_command("FREQ " + std::to_string(freq), false);
    send_command("VOLT " + std::to_string(amp), false);
    send_command("PWID " + std::to_string(pulse_width), false);  // Set pulse width
    send_command("RISD " + std::to_string(edge_time), false);  // Set edge time
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
  }

  void set_noise_wave(double amp) {
    send_command("FUNC NOIS", false);
    send_command("VOLT " + std::to_string(amp), false);
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
  }

  void configure_arbitrary_wave(double freq, double amp, const std::string &waveform_points) {
    send_command("FUNC ARB", false);
    send_command("FREQ " + std::to_string(freq), false);
    send_command("VOLT " + std::to_string(amp), false);
    send_command("DATA:ARB " + waveform_points, false);
    enable_output();  // Enable output after setting the waveform
    wait_for_operation_complete();
    check_device_status();
  }

  void prompt_user() {
    std::string waveform_type;
    while (true) {
      RCLCPP_INFO(this->get_logger(), "Enter waveform type (SIN/SQUARE/RAMP/PULSE/NOISE/ARB/EXIT):");
      std::getline(std::cin, waveform_type);
      std::transform(waveform_type.begin(), waveform_type.end(), waveform_type.begin(), ::toupper);

      if (waveform_type == "EXIT") {
        RCLCPP_INFO(this->get_logger(), "Exiting...");
        rclcpp::shutdown();
        break;
      } else {
        configure_waveform(waveform_type);
      }
    }
  }

  void configure_waveform(const std::string &waveform_type) {
    double freq, amp, duty_cycle, symmetry, pulse_width, edge_time;
    std::string waveform_points;

    if (waveform_type == "SIN") {
        RCLCPP_INFO(this->get_logger(), "Enter frequency (Hz):");
        std::cin >> freq;
        std::cin.ignore();  // Ignore the newline character after reading freq
        RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
        std::cin >> amp;
        std::cin.ignore();  // Ignore the newline character after reading amp
        set_sine_wave(freq, amp);
    } else if (waveform_type == "SQUARE") {
        RCLCPP_INFO(this->get_logger(), "Enter frequency (Hz):");
        std::cin >> freq;
        std::cin.ignore();  // Ignore the newline character after reading freq
        RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
        std::cin >> amp;
        std::cin.ignore();  // Ignore the newline character after reading amp
        RCLCPP_INFO(this->get_logger(), "Enter duty cycle (0-100%%):");
        std::cin >> duty_cycle;
        std::cin.ignore();  // Ignore the newline character after reading duty_cycle
        set_square_wave(freq, amp, duty_cycle);
    } else if (waveform_type == "RAMP") {
        RCLCPP_INFO(this->get_logger(), "Enter frequency (Hz):");
        std::cin >> freq;
        std::cin.ignore();  // Ignore the newline character after reading freq
        RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
        std::cin >> amp;
        std::cin.ignore();  // Ignore the newline character after reading amp
        RCLCPP_INFO(this->get_logger(), "Enter symmetry (0-100%%):");
        std::cin >> symmetry;
        std::cin.ignore();  // Ignore the newline character after reading symmetry
        set_ramp_wave(freq, amp,symmetry);
    }  else if (waveform_type == "PULSE") {
        RCLCPP_INFO(this->get_logger(), "Enter frequency (Hz):");
        std::cin >> freq;
        std::cin.ignore();  // Ignore the newline character after reading freq
        RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
        std::cin >> amp;
        std::cin.ignore();  // Ignore the newline character after reading amp
        RCLCPP_INFO(this->get_logger(), "Enter pulse width (s):");
        std::cin >> pulse_width;
        std::cin.ignore();  // Ignore the newline character after reading duty_cycle
        RCLCPP_INFO(this->get_logger(), "Enter edge time (s):");
        std::cin >> edge_time;
        std::cin.ignore();  // Ignore the newline character after reading duty_cycle
        set_pulse_wave(freq, amp, pulse_width, edge_time);
    } else if (waveform_type == "NOISE") {
      RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
      std::cin >> amp;
      std::cin.ignore();  // Ignore the newline character after reading amp
      set_noise_wave(amp);
    } else if (waveform_type == "ARB") {
      RCLCPP_INFO(this->get_logger(), "Enter frequency (Hz):");
      std::cin >> freq;
      std::cin.ignore();  // Ignore the newline character after reading freq
      RCLCPP_INFO(this->get_logger(), "Enter amplitude (V):");
      std::cin >> amp;
      std::cin.ignore();  // Ignore the newline character after reading amp
      RCLCPP_INFO(this->get_logger(), "Enter waveform points (comma separated, e.g. 0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5):");
      std::getline(std::cin, waveform_points);  // Now this will work correctly
      configure_arbitrary_wave(freq, amp, waveform_points);
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
