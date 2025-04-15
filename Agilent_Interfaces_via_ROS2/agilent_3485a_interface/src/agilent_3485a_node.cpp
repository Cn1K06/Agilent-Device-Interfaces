#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <cstring>
#include <iostream>
#include <string>
#include <thread> // For std::this_thread::sleep_for

class Agilent3485ANode : public rclcpp::Node {
public:
  Agilent3485ANode() : Node("agilent_3485a_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("measurements", 10);

    // Initialize the socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening socket");
      return;
    }

    server = gethostbyname("172.17.2.21"); // IP address
    if (server == NULL) {
      RCLCPP_ERROR(this->get_logger(), "Error, no such host");
      return;
    }

    std::memset((char *) &server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    std::memcpy((char *)&server_addr.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
    server_addr.sin_port = htons(1234); // Port number

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error connecting");
      return;
    }

    // Clear any existing data in the buffer
    clear_buffer();

    // First, learn the device's identity
    identity_ = send_command("*ID?");
    RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&Agilent3485ANode::timer_callback, this));
  }

  ~Agilent3485ANode() {
    close(sockfd);
  }

private:
  int sockfd;
  struct sockaddr_in server_addr;
  struct hostent *server;
  std::string identity_;

  void clear_buffer() {
    char buffer[1024];
    while (read(sockfd, buffer, sizeof(buffer)) > 0);
  }

  std::string send_command(const std::string &command) {
    std::string request = command + "\n";
    if (write(sockfd, request.c_str(), request.size()) < 0) {
      return "Error: Cannot write to socket";
    }

    // Add a small delay to ensure the device processes the command
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    char buffer[1024];
    int n = read(sockfd, buffer, sizeof(buffer));
    if (n < 0) {
      return "Error: Cannot read from socket";
    }

    return std::string(buffer, n);
  }

  void timer_callback() {
    std::string voltage = send_command("MEAS:VOLT:DC?");
    auto message = std_msgs::msg::String();
    message.data = "Voltage: " + voltage;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agilent3485ANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
