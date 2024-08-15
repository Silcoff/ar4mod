#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/component/component.hpp>

using namespace ftxui;

class ControlDebugger : public rclcpp::Node {
public:
  ControlDebugger() : Node("control_debugger") {
    // Create subscription to the "topic"
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          topic_callback(msg);
        });

    // Start the FTXUI interface in a separate thread
    ui_thread_ = std::thread([this]() {
      auto screen = ScreenInteractive::FitComponent();
      auto renderer = Renderer([this] {
        return vbox({
            text("Listening to ROS2 topic..."),
            separator(),
            text(message_)
        }) | border;
      });

      screen.Loop(renderer);
    });
  }

  ~ControlDebugger() {
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    message_ = msg->data;
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string message_ = "Waiting for messages...";
  std::thread ui_thread_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlDebugger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

