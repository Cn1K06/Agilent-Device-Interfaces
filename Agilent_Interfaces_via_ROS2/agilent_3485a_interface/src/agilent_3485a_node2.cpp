#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <string>
#include <memory>
#include <boost/asio.hpp> 

using boost::asio::ip::tcp;  

class Agilent3485ANode : public rclcpp::Node {
public:
  Agilent3485ANode()
  : Node("agilent_3485a_node2"),
    ip_address_("172.17.2.21"),
    port_(1234)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("measurements", 10);

    try {
        connect_to_device();
      
        // Reset und clear status
        send_command("*RESET");
        send_command("*CLS");
      
        identity_ = send_command("*ID?");
        RCLCPP_INFO(this->get_logger(), "Device Identity: '%s'", identity_.c_str());
        
        timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&Agilent3485ANode::timer_callback, this)
        );
      } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", e.what());
      }
  }

private:
  std::string ip_address_;
  int port_;
  std::string identity_;

  boost::asio::io_context io_context_; 
  std::unique_ptr<tcp::socket> socket_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void connect_to_device() {
    tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(ip_address_, std::to_string(port_));
    socket_ = std::make_unique<tcp::socket>(io_context_);
    boost::asio::connect(*socket_, endpoints);
    RCLCPP_INFO(this->get_logger(), "Connected to Agilent 3485A at %s:%d", ip_address_.c_str(), port_);
  }

  std::string send_command(const std::string &command) {
    std::string cmd = command + "\n";
    boost::asio::write(*socket_, boost::asio::buffer(cmd));

    boost::asio::streambuf response;
    boost::asio::read_until(*socket_, response, "\r\n");

    std::istream response_stream(&response);
    std::string result;
    std::getline(response_stream, result);

    return result.empty() ? "No response" : result;
  }

  void timer_callback() {
    try {
      std::string voltage = send_command("MEAS:VOLT:DC?");
      auto message = std_msgs::msg::String();
      message.data = "Voltage: " + voltage;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Communication error: %s", e.what());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Agilent3485ANode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
