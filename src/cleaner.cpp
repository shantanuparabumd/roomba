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
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    timer_ = this->create_wall_timer(100ns, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

  }

 private:
  // Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  
  size_t count_;
  
/**
 * @brief Method that runs after every 1000ms (Set as base frequency)
 * 
 */
  void timer_callback() {
    // C++ stream style
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "The Node Has been Setup");
    auto command = geometry_msgs::msg::Twist();
    command.linear.x=0.0;
    // RCLCPP_INFO(this->get_logger(), "808X ROS2: '%f'", command.linear.x);
    // publisher_->publish(command);
    

  }
  void topic_callback(const sensor_msgs::msg::LaserScan & msg) const {
    if(msg.ranges[0]<1){
        
    }
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.ranges[0]);
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