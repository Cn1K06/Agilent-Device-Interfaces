#include "rclcpp/rclcpp.hpp"
#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <algorithm>
#include <ctime>

class AgilentInterfaceNode : public rclcpp::Node {
public:
    AgilentInterfaceNode()
    : Node("agilent_3485a_node5"), ip_("172.17.2.21"), port_(1234), sockfd_(-1), measure_active_(false), should_exit_(false)
    {
        if (connectDevice()) {
            RCLCPP_INFO(get_logger(), "Connected to Agilent %s", getIdentity().c_str());
            
            reset_instrument();
            RCLCPP_INFO(get_logger(), "Instrument is reset.");

            clear_status();
            RCLCPP_INFO(get_logger(), "Status cleared.");

            commandLoop();
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to connect to Agilent 3458A");
        }
    }

    ~AgilentInterfaceNode() {
        disconnectDevice();
    }

private:
    std::string ip_;
    int port_;
    int sockfd_;
    std::string measurement_cmd_;
    std::string measurement_type_;
    std::atomic<bool> measure_active_;
    std::atomic<bool> should_exit_;

    bool connectDevice() {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0) {
            perror("Socket creation failed");
            return false;
        }
        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port_);
        inet_pton(AF_INET, ip_.c_str(), &servaddr.sin_addr);
        if (connect(sockfd_, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            perror("Connection failed");
            return false;
        }
        return true;
    }

    void disconnectDevice() {
        if (sockfd_ >= 0) {
            close(sockfd_);
            sockfd_ = -1;
        }
    }

    bool sendCommand(const std::string& cmd) {
        std::string command = cmd + "\n";
        ssize_t sent = send(sockfd_, command.c_str(), command.length(), 0);
        return sent == static_cast<ssize_t>(command.length());
    }

    void clearBuffer() {
        char tmp[1024];
        recv(sockfd_, tmp, sizeof(tmp), MSG_DONTWAIT);
    }

    std::string query(const std::string& cmd) {
        clearBuffer();
        if (!sendCommand(cmd)) return "";
        char buffer[1024] = {0};
        ssize_t len = recv(sockfd_, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) return "";
        buffer[len] = '\0';
        return std::string(buffer);
    }

    std::string getIdentity() {
        return query("ID?");
    }

    void reset_instrument() {
        sendCommand("RESET");
    }

    void clear_status() {
        sendCommand("*CLS");
    }

    void commandLoop() {
        std::string command;
        while (rclcpp::ok()) {
            std::cout << "\nEnter command (DC/AC/CURR_DC/CURR_AC/RES/FREQ/STOP/EXIT): ";
            std::getline(std::cin, command);
            std::transform(command.begin(), command.end(), command.begin(), ::toupper);

            if (command == "EXIT") {
                RCLCPP_INFO(get_logger(), "Exiting the program.");
                rclcpp::shutdown();
                break;
            }

            if (command == "DC" || command == "AC" || command == "CURR_DC" || command == "CURR_AC" || command == "RES" || command == "FREQ") {
                configureMeasurement(command);
                measure_active_ = true;
                RCLCPP_INFO(get_logger(), "Starting continuous %s measurement. Type STOP to finish.", measurement_type_.c_str());

                std::thread input_thread([&]() {
                    std::string input;
                    while (measure_active_) {
                        std::getline(std::cin, input);
                        std::transform(input.begin(), input.end(), input.begin(), ::toupper);
                        if (input == "STOP") {
                            measure_active_ = false;
                        }
                    }
                });

                while (measure_active_ && rclcpp::ok()) {
                    auto result = query(measurement_cmd_);

                    std::time_t now = std::time(nullptr);
                    char timestamp[100];
                    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

                    RCLCPP_INFO(get_logger(), "%s at %s: %s", measurement_type_.c_str(), timestamp, result.c_str());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                input_thread.join();
                RCLCPP_INFO(get_logger(), "Measurement stopped.");
            } else {
                RCLCPP_WARN(get_logger(), "Invalid command.");
            }
        }
    }

    void configureMeasurement(const std::string& type) {
        if (type == "DC") {
            sendCommand("DCV AUTO");
            measurement_cmd_ = "TARM SGL, DCV?";
            measurement_type_ = "DC Voltage";
        } else if (type == "AC") {
            sendCommand("ACV AUTO");
            measurement_cmd_ = "TARM SGL, ACV?";
            measurement_type_ = "AC Voltage";
        } else if (type == "CURR_DC") {
            sendCommand("DCI AUTO");
            measurement_cmd_ = "TARM SGL, DCI?";
            measurement_type_ = "DC Current";
        } else if (type == "CURR_AC") {
            sendCommand("ACI AUTO");
            measurement_cmd_ = "TARM SGL, ACI?";
            measurement_type_ = "AC Current";
        } else if (type == "RES") {
            sendCommand("OHMF AUTO");
            measurement_cmd_ = "TARM SGL, OHMF?";
            measurement_type_ = "Resistance";
        } else if (type == "FREQ") {
            sendCommand("FREQ AUTO");
            measurement_cmd_ = "TARM SGL, FREQ?";
            measurement_type_ = "Frequency";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgilentInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
