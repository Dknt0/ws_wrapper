#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocket.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils/js_temp.hpp"

class TwistToWebSocketNode : public rclcpp::Node {
 public:
  TwistToWebSocketNode() : Node("twist_to_websocket_node") {
    ix::initNetSystem();
    web_socket_.setUrl(url_);
    web_socket_.setOnMessageCallback(
        [this](const ix::WebSocketMessagePtr& msg) {
          if (msg->type == ix::WebSocketMessageType::Message) {
          }
        });
    web_socket_.start();

    twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          // RCLCPP_INFO(this->get_logger(),
          //             "Received Twist - Linear: [x: %.2f, y: %.2f, z: %.2f],
          //             " "Angular: [x: %.2f, y: %.2f, z: %.2f]",
          //             msg->linear.x, msg->linear.y, msg->linear.z,
          //             msg->angular.x, msg->angular.y, msg->angular.z);

          web_socket_.send(VelocityMsg("set_vel_x", msg->linear.x));
          web_socket_.send(VelocityMsg("set_vel_y", msg->linear.y));
          web_socket_.send(VelocityMsg("set_vel_w", msg->angular.z));
        });
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      twist_subscription_;
  ix::WebSocket web_socket_;
  std::string url_ = "ws://127.0.0.1:12888/console";
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistToWebSocketNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
