/**
 * @file cleaner.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief A Algorithm for turtlebot to travel through maze
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;



class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()  // Object for node is created using a constructor
  : Node("cleaner"), count_(0) {
    
  // Creating a Publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    timer_ = this->create_wall_timer(100ns, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

  }

 private:
  // Variables
  std::string Message = "Shantanu";
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  
  size_t count_;
  
/**
 * @brief Method that runs after every 1000ms (Set as base frequency)
 * 
 */
  void timer_callback() {
    // C++ stream style
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Node Has been Setup");
    auto message = std_msgs::msg::String();
    message.data = "Hello, " +Message + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "808X ROS2: '%s'", message.data.c_str());
    publisher_->publish(message);
    

  }
  void topic_callback(const sensor_msgs::msg::LaserScan & msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.angle_min);
  }
};

/**
 * @brief Main function Entrypoint for program
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}