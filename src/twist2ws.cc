#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocket.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

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
        "/robot_cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          // RCLCPP_INFO(this->get_logger(),
          //             "Received Twist - Linear: [x: %.2f, y: %.2f, z: %.2f],
          //             " "Angular: [x: %.2f, y: %.2f, z: %.2f]",
          //             msg->linear.x, msg->linear.y, msg->linear.z,
          //             msg->angular.x, msg->angular.y, msg->angular.z);

          web_socket_.send(VelocityMsg("set_vel_x", msg->linear.x));
          web_socket_.send(VelocityMsg("set_vel_y", msg->linear.y));
          web_socket_.send(VelocityMsg("set_vel_w", msg->angular.z));
        });
    int_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "/robot_action", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
          if (msg->data == 2) {
            web_socket_.send(ButtonMsg<"enable_record", "2">().value);
            web_socket_.send(ButtonMsg<"power_on", "1">().value);
            web_socket_.send(ButtonMsg<"start", "2">().value);
            RCLCPP_INFO(this->get_logger(), "Sent power_on and start commands");
          } else if (msg->data == 3) {
            web_socket_.send(ButtonMsg<"init_pose", "2">().value);
            RCLCPP_INFO(this->get_logger(), "Sent init_pos commands");
          } else if (msg->data == 4) {
            web_socket_.send(ButtonMsg<"run_policy", "2">().value);
            RCLCPP_INFO(this->get_logger(), "Sent run_policy commands");
          } else if (msg->data == 1) {
            web_socket_.send(ButtonMsg<"enable_warking_policy", "2">().value);
            RCLCPP_INFO(this->get_logger(),
                        "Sent enable_warking_policy commands");
          } else if (msg->data == 0) {
            web_socket_.send(ButtonMsg<"stop", "1">().value);
          }
        });
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      twist_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_subscription_;
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
