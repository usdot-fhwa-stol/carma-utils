// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "rclcpp/rclcpp.hpp"

// Based off of: https://github.com/sukha-cn/turtlesim-ros2/blob/master/tutorials/teleop_turtle_key.cpp

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36

int kfd = 0;
struct termios cooked, raw;

class CarmaEventPub
{
public:
  explicit CarmaEventPub(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<carma_msgs::msg::SystemAlert>::SharedPtr event_pub_;
};

CarmaEventPub::CarmaEventPub(std::shared_ptr<rclcpp::Node> nh)
: nh_(nh)
{
  event_pub_ = nh_->create_publisher<carma_msgs::msg::SystemAlert>("/system_alert", 1);
}

void CarmaEventPub::keyLoop()
{
  // Put the console into raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts(
    "Use 1(CAUTION),2(WARNING),3(FATAL),4(NOT_READY),5(DRIVERS_READY),"
    "6(SHUTDOWN) keys to issue system alerts");

  for (;; ) {
    // Get the next event from the keyboard
    char c;
    if (::read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    carma_msgs::msg::SystemAlert new_msg;

    switch (c) {
      case KEYCODE_1:
        std::cout << "CAUTION" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::CAUTION;
        break;
      case KEYCODE_2:
        std::cout << "WARNING" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::WARNING;
        break;
      case KEYCODE_3:
        std::cout << "FATAL" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::FATAL;
        break;
      case KEYCODE_4:
        std::cout << "NOT_READY" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::NOT_READY;
        break;
      case KEYCODE_5:
        std::cout << "DRIVERS_READY" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::DRIVERS_READY;
        break;
      case KEYCODE_6:
        std::cout << "SHUTDOWN" << std::endl;
        new_msg.type = carma_msgs::msg::SystemAlert::SHUTDOWN;
        break;
      default:
        return;
    }

    new_msg.description = "dummy_message";
    event_pub_->publish(new_msg);
  }
}

void quit(int sig)
{
  sig = sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("carma_event_pub");
  CarmaEventPub event_handler(node);
  signal(SIGINT, quit);
  event_handler.keyLoop();

  return 0;
}
