/**
 * @file simple_bag_recorder.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Recorder Node
 * @version 0.1
 * @date 2022-12-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node {
 public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    writer_->open("my_bag");

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

 private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
    rclcpp::Time time_stamp = this->now();

    writer_->write(msg, "cmd_vel", "geometry_msgs/msg/Twist", time_stamp);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
