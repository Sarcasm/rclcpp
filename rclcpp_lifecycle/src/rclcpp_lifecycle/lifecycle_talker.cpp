// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp::node::lifecycle::LifecycleNode
{
public:
  explicit MyLifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp::node::lifecycle::LifecycleNode(node_name, intra_process_comms)
  {
    msg_ = std::make_shared<std_msgs::msg::String>();

    pub_ = this->create_publisher<std_msgs::msg::String>("chatter");

    timer_ = this->create_wall_timer(1_s, std::bind(&MyLifecycleNode::publish, this));
  }

  void publish()
  {
    static size_t count = 0;
    msg_->data = "HelloWorld" + std::to_string(++count);
    pub_->publish(msg_);
  }

  bool on_configure()
  {
    printf("Going to configure my Node\n");
    return true;
  }

  bool on_activate()
  {
    rclcpp::node::lifecycle::LifecycleNode::enable_communication();
    printf("Going to activate my node\n");
    return true;
  }

  bool on_deactivate()
  {
    rclcpp::node::lifecycle::LifecycleNode::disable_communication();
    printf("Going to deactivate my node\n");
    return true;
  }

private:
  std::shared_ptr<std_msgs::msg::String> msg_;
  std::shared_ptr<rclcpp::publisher::LifecyclePublisher<std_msgs::msg::String>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<MyLifecycleNode> lc_node = std::make_shared<MyLifecycleNode>("lc_talker");

  rclcpp::lifecycle::LifecycleManager lm;
  lm.add_node_interface(lc_node);

  exe.add_node(lc_node);

  auto time_out_lambda = []() -> int {
      std::this_thread::sleep_for(std::chrono::seconds(10));
      return 0;
    };

  // configure
  // dummy mockup for now!
  lm.configure("my_node1");
  std::shared_future<int> time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  lm.activate("my_node1");
  time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  lm.deactivate("my_node1");
  time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  return 0;
}
