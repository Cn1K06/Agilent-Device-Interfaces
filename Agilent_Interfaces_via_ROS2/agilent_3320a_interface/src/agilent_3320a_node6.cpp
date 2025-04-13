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

    // Prompt user for waveform or modulation type
    prompt_user();

    disable_output();
    reset_device();

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

  void set_modulation_am(const std::string &carrier_waveform, const std::string &modulating_waveform, double carrier_freq, double carrier_amp, double mod_freq, double depth) {
    send_command("FUNC " + carrier_waveform, false);  // Set carrier waveform
    send_command("FREQ " + std::to_string(carrier_freq), false);  // Set carrier frequency
    send_command("VOLT " + std::to_string(carrier_amp), false);  // Set carrier amplitude
    send_command("AM:STATE ON");
    send_command("AM:SOUR INT");
    send_command("AM:INT:FUNC " + modulating_waveform, false);  // Set modulating waveform
    send_command("AM:INT:FREQ " + std::to_string(mod_freq), false);  // Set modulation frequency
    send_command("AM:DEPTH " + std::to_string(depth), false);  // Set modulation depth
    enable_output();  // Enable output after setting the modulation
    wait_for_operation_complete();  // Wait until the operation is complete
}


  void set_modulation_fm(const std::string &carrier_waveform, const std::string &modulating_waveform, double carrier_freq, double carrier_amp, double mod_freq, double deviation) {
    send_command("FUNC " + carrier_waveform, false);  // Set carrier waveform
    send_command("FREQ " + std::to_string(carrier_freq), false);  // Set carrier frequency
    send_command("VOLT " + std::to_string(carrier_amp), false);  // Set carrier amplitude
    send_command("FM:STATE ON");
    send_command("FM:SOUR INT");
    send_command("FM:INT:FUNC " + modulating_waveform, false);  // Set modulating waveform
    send_command("FM:FREQ " + std::to_string(mod_freq), false);  // Set modulation frequency
    send_command("FM:DEV " + std::to_string(deviation), false);  // Set frequency deviation
    enable_output();  // Enable output after setting the modulation
    wait_for_operation_complete();  // Wait until the operation is complete
}


  void set_modulation_pm(const std::string &carrier_waveform, const std::string &modulating_waveform, double carrier_freq, double carrier_amp, double mod_freq, double deviation) {
    send_command("FUNC " + carrier_waveform, false);  // Set carrier waveform
    send_command("FREQ " + std::to_string(carrier_freq), false);  // Set carrier frequency
    send_command("VOLT " + std::to_string(carrier_amp), false);  // Set carrier amplitude
    send_command("PM:STATE ON");
    send_command("PM:SOUR INT");
    send_command("PM:INT:FUNC " + modulating_waveform, false);  // Set modulating waveform
    send_command("PM:FREQ " + std::to_string(mod_freq), false);  // Set modulation frequency
    send_command("PM:DEV " + std::to_string(deviation), false);  // Set phase deviation
    enable_output();  // Enable output after setting the modulation
    wait_for_operation_complete();  // Wait until the operation is complete
}


  void set_modulation_fsk(const std::string &carrier_waveform, double carrier_freq, double carrier_amp, double shift_freq, double rate) {
      send_command("FUNC " + carrier_waveform, false);  // Set carrier waveform
      send_command("FREQ " + std::to_string(carrier_freq), false);  // Set carrier frequency
      send_command("VOLT " + std::to_string(carrier_amp), false);  // Set carrier amplitude
      send_command("FSK:STATE ON");
      send_command("FSK:SOUR INT");
      send_command("FSK:FREQ " + std::to_string(shift_freq), false);  // Set frequency shift
      send_command("FSK:RATE " + std::to_string(rate), false);  // Set modulation rate
      enable_output();  // Enable output after setting the modulation
      wait_for_operation_complete();  // Wait until the operation is complete
  }


  void set_modulation_pwm(const std::string &carrier_waveform, const std::string &modulating_waveform, double carrier_freq, double carrier_amp, double mod_freq, double deviation) {
      send_command("FUNC " + carrier_waveform, false);  // Set carrier waveform
      send_command("FREQ " + std::to_string(carrier_freq), false);  // Set carrier frequency
      send_command("VOLT " + std::to_string(carrier_amp), false);  // Set carrier amplitude
      send_command("PWM:STATE ON");
      send_command("PWM:SOUR INT");
      send_command("PWM:INT:FUNC" + modulating_waveform, false);  // Set modulating waveform
      send_command("PWM:FREQ " + std::to_string(mod_freq), false);  // Set modulation frequency
      send_command("PWM:DEV " + std::to_string(deviation), false);  // Set duty cycle
      enable_output();  // Enable output after setting the modulation
      wait_for_operation_complete();  // Wait until the operation is complete
  }


  void set_sweep(const std::string &sweep_type, double start_freq, double stop_freq, double sweep_time){
    send_command("SWE:STATE ON", false);
    send_command("FUNC SIN", false);
    send_command("FREQ:STAR " + std::to_string(start_freq), false);
    send_command("FREQ:STOP " + std::to_string(stop_freq),false);
    send_command("SWE:TIME " + std::to_string(sweep_time),false);
    if (sweep_type == "LIN") {
        send_command("SWE:SPAC LIN", false ); // linear sweep
    } else if (sweep_type == "LOG"){
        send_command("SWE:SPAC LOG", false ); // logarithmic sweep
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid Sweep type. Please enter a valid Sweep type lin or log.");
    }
  }

  void set_burst(const std::string &trigger_source, double carrier_freq, double carrier_amp, double burst_count, double burst_period){
    send_command("BURST:STATE ON", false);
    send_command("FUNC SIN", false);
    send_command("FREQ " + std::to_string(carrier_freq), false);
    send_command("VOLT " + std::to_string(carrier_amp),false);
    send_command("BURST:NCYC " + std::to_string(burst_count),false);
    send_command("BURST:INT:PER " + std::to_string(burst_period),false);

    if (trigger_source == "INT") {
        send_command("BURST:TRIG:SOUR INT", false ); // Internal trigger
    } else if (trigger_source == "EXT"){
        send_command("BURST:TRIG:SOUR EXT", false ); // External trigger 
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid trigger source type. Please enter a valid trigger source type int or ext.");
    }
  }

  void prompt_user() {
    std::string waveform_type;
    std::string mod_type;
    while (true) {
        RCLCPP_INFO(this->get_logger(), "Enter waveform type (SIN/SQUARE/RAMP/PULSE/NOISE/ARB/MODULATION/SWEEP/BURST/EXIT):");
        std::getline(std::cin, waveform_type);
        std::transform(waveform_type.begin(), waveform_type.end(), waveform_type.begin(), ::toupper);

        if (waveform_type == "EXIT") {
            RCLCPP_INFO(this->get_logger(), "Exiting...");
            rclcpp::shutdown();  // Proper shutdown of ROS node
            break;
        } else if (waveform_type == "MODULATION") {
            RCLCPP_INFO(this->get_logger(), "Enter modulation type (AM/FM/PM/FSK/PWM):");
            std::getline(std::cin, mod_type);
            std::transform(mod_type.begin(), mod_type.end(), mod_type.begin(), ::toupper);

            // Ensure valid modulation type before proceeding
            if (mod_type == "AM" || mod_type == "FM" || mod_type == "PM" || mod_type == "FSK" || mod_type == "PWM") {
                configure_modulation(mod_type);  // Call modulation configuration
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid modulation type. Please enter AM, FM, PM, FSK, or PWM.");
            }
        } else if (waveform_type == "SIN" || waveform_type == "SQUARE" || waveform_type == "RAMP" ||
                  waveform_type == "PULSE" || waveform_type == "NOISE" || waveform_type == "ARB") {
            configure_waveform(waveform_type);  // Configure other waveforms
        } else if (waveform_type == "SWEEP") {
            configure_sweep();  // Call the sweep configuration
        } else if (waveform_type == "BURST") {
            configure_burst();  // Call the burst configuration
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid waveform type. Please enter a valid waveform or EXIT.");
        }
    }
  }

  void configure_sweep() {
    double start_freq, stop_freq, sweep_time;
    std::string sweep_type;

    RCLCPP_INFO(this->get_logger(), "Enter start frequency (Hz):");
    std::cin >> start_freq;
    RCLCPP_INFO(this->get_logger(), "Enter stop frequency (Hz):");
    std::cin >> stop_freq;
    RCLCPP_INFO(this->get_logger(), "Enter sweep time (seconds):");
    std::cin >> sweep_time;
    std::cin.ignore(); // Ignore newline left in the buffer
    RCLCPP_INFO(this->get_logger(), "Enter sweep type (LIN/LOG):");
    std::getline(std::cin, sweep_type);
    std::transform(sweep_type.begin(), sweep_type.end(), sweep_type.begin(), ::toupper);
    
    set_sweep(sweep_type, start_freq, stop_freq, sweep_time);  // Call sweep configuration
  }

  void configure_burst() {
    double carrier_freq, carrier_amp, burst_count, burst_period;
    std::string trigger_source;
    
    RCLCPP_INFO(this->get_logger(), "Enter carrier frequency (Hz):");
    std::cin >> carrier_freq;
    RCLCPP_INFO(this->get_logger(), "Enter carrier amplitude (Vpp):");
    std::cin >> carrier_amp;
    RCLCPP_INFO(this->get_logger(), "Enter number of burst cycles:");
    std::cin >> burst_count;
    RCLCPP_INFO(this->get_logger(), "Enter burst period (seconds):");
    std::cin >> burst_period;
    std::cin.ignore(); // Ignore newline left in the buffer
    RCLCPP_INFO(this->get_logger(), "Enter trigger source (INT/EXT):");
    std::getline(std::cin, trigger_source);
    std::transform(trigger_source.begin(), trigger_source.end(), trigger_source.begin(), ::toupper);

    set_burst(trigger_source, carrier_freq, carrier_amp, burst_count, burst_period);  // Call burst configuration
  }

  void configure_modulation(const std::string &mod_type) {
    std::string carrier_waveform, modulating_waveform;
    double carrier_freq, carrier_amp, mod_freq, depth, deviation, shift_freq, rate;

    // Get carrier waveform from user
    RCLCPP_INFO(this->get_logger(), "Enter carrier waveform (SIN/SQUARE/RAMP):");
    std::cin >> carrier_waveform;
    std::cin.ignore();

    if (mod_type == "PWM") {
      RCLCPP_INFO(this->get_logger(), "PWM modulation requires PULSE waveform as the carrier.");
      carrier_waveform = "PULS";  // Set carrier waveform to PULS for PWM
      RCLCPP_INFO(this->get_logger(), "Choose modulating waveform (SIN/SQUARE/RAMP/TRIANGLE/NRAMP):");
      std::cin >> modulating_waveform;
      std::cin.ignore();
    } else if (mod_type == "FSK") {
      RCLCPP_INFO(this->get_logger(), "FSK modulation selected. Modulating waveform is not needed for FSK.");
      RCLCPP_INFO(this->get_logger(), "Choose carrier waveform (SIN/SQUARE/RAMP):");
      std::cin >> carrier_waveform;
      std::cin.ignore();
    } else {
      RCLCPP_INFO(this->get_logger(), "Choose modulating waveform (SIN/SQUARE/RAMP/TRIANGLE/NRAMP):");
      std::cin >> modulating_waveform;
      std::cin.ignore();
    }

    RCLCPP_INFO(this->get_logger(), "Enter carrier frequency (Hz):");
    std::cin >> carrier_freq;
    std::cin.ignore();
    RCLCPP_INFO(this->get_logger(), "Enter carrier amplitude (Vpp):");
    std::cin >> carrier_amp;
    std::cin.ignore();

    if (mod_type == "AM") {
      RCLCPP_INFO(this->get_logger(), "Enter modulation frequency (Hz):");
      std::cin >> mod_freq;
      std::cin.ignore();
      RCLCPP_INFO(this->get_logger(), "Enter modulation depth (%%):");
      std::cin >> depth;
      std::cin.ignore();
      set_modulation_am(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, depth);
    } else if (mod_type == "FM") {
      RCLCPP_INFO(this->get_logger(), "Enter modulation frequency (Hz):");
      std::cin >> mod_freq;
      std::cin.ignore();
      RCLCPP_INFO(this->get_logger(), "Enter frequency deviation (Hz):");
      std::cin >> deviation;
      std::cin.ignore();
      set_modulation_fm(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation);
    } else if (mod_type == "PM") {
      RCLCPP_INFO(this->get_logger(), "Enter modulation frequency (Hz):");
      std::cin >> mod_freq;
      std::cin.ignore();
      RCLCPP_INFO(this->get_logger(), "Enter phase deviation (degrees):");
      std::cin >> deviation;
      std::cin.ignore();
      set_modulation_pm(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation);
    } else if (mod_type == "FSK") {
      RCLCPP_INFO(this->get_logger(), "Enter shift frequency (Hz):");
      std::cin >> shift_freq;
      std::cin.ignore();
      RCLCPP_INFO(this->get_logger(), "Enter FSK rate (Hz):");
      std::cin >> rate;
      std::cin.ignore();
      set_modulation_fsk(carrier_waveform, carrier_freq, carrier_amp, shift_freq, rate);
    } else if (mod_type == "PWM") {
      RCLCPP_INFO(this->get_logger(), "Enter modulation frequency (Hz):");
      std::cin >> mod_freq;
      std::cin.ignore();
      RCLCPP_INFO(this->get_logger(), "Enter pulse width deviation (%%):");
      std::cin >> deviation;
      std::cin.ignore();
      set_modulation_pwm(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid modulation type.");
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

