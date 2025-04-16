#include "rclcpp/rclcpp.hpp"
#include <string>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class AgilentInterfaceNode : public rclcpp::Node {
public:
    AgilentInterfaceNode()
    : Node("agilent_3485a_node3"), ip_("172.17.2.21"), port_(1234), sockfd_(-1)
    {
        if (connectDevice()) {
            RCLCPP_INFO(get_logger(), "Connected to Agilent %s", getIdentity().c_str());
            reset_instrument();
            RCLCPP_INFO(get_logger(), "Instrument is reset.");
            clear_status();
            RCLCPP_INFO(get_logger(), "Status cleared.");
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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgilentInterfaceNode>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
