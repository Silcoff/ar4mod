#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/dom/table.hpp>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"

using std::placeholders::_1;

    std::string reset_position;
class ControlDebugger : public rclcpp::Node
{
  public:
    ControlDebugger(): Node("joint_display"){
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&ControlDebugger::topic_callback, this, _1));
      trajectory_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/joint_trajectory_controller/controller_state", 10, std::bind(&ControlDebugger::trajectory_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
      using namespace ftxui;
       // Define the document
      Element document =
      hbox({
        text(msg->data.c_str())   | border,
      });
   
      auto screen = Screen::Create(Dimension::Full(),Dimension::Fit(document));
      Render(screen, document);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      std::cout << reset_position << screen.ToString() << std::flush;
      reset_position = screen.ResetPosition();
      screen.Clear();
      // screen.Print();
    }
    void trajectory_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) const{
      using namespace ftxui;
    // Accessing the desired state
    auto desired_positions = msg->feedback.positions;
    auto desired_velocities = msg->feedback.velocities;

    // Accessing the actual state
    auto actual_positions = msg->reference.positions;
    auto actual_velocities = msg->reference.velocities;

    // Accessing the error state
    auto position_errors = msg->error.positions;
    auto velocity_errors = msg->error.velocities;
    
    std::vector<std::string> position_errors_str;
    for (const auto &error : position_errors) {
      position_errors_str.push_back(std::to_string(error*(180/3.14)));
    }

    std::vector<std::string> position_desired_str;
    for (const auto &desired : desired_positions) {
      position_desired_str.push_back(std::to_string(desired*(180/3.14)));
    }

    std::vector<std::string> position_actual_str;
    for (const auto &actual : actual_positions) {
      position_actual_str.push_back(std::to_string(actual*(180/3.14)));
    }


  auto table = Table({
      {"error" ,"feedback", "reference"},
      {position_errors_str[0], position_desired_str[0],  position_actual_str[0]}, 
      {position_errors_str[1], position_desired_str[1],  position_actual_str[1]}, 
      {position_errors_str[2], position_desired_str[2],  position_actual_str[2]}, 
      {position_errors_str[3], position_desired_str[3],  position_actual_str[3]}, 
      {position_errors_str[4], position_desired_str[4],  position_actual_str[4]}, 
      {position_errors_str[5], position_desired_str[5],  position_actual_str[5]}, 
  });

  table.SelectAll().Border(LIGHT);

  // Add border around the first column.
  table.SelectColumn(0).Border(LIGHT);
  table.SelectColumn(1).Border(LIGHT);

  // Make first row bold with a double border.
  table.SelectRow(0).Decorate(bold);
  table.SelectRow(0).SeparatorVertical(LIGHT);
  table.SelectRow(0).Border(DOUBLE);



  auto document = table.Render();
      // Element document =
      // hbox({
      //   text(position_actual_str[3])   | border,
      // });
   
      auto screen = Screen::Create(Dimension::Fit(document));
      Render(screen, document);
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      std::cout << reset_position << screen.ToString() << std::flush;
      reset_position = screen.ResetPosition();
      screen.Clear();
      // screen.Print();
    }



    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr trajectory_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlDebugger>());
  rclcpp::shutdown();
  return 0;
} 
