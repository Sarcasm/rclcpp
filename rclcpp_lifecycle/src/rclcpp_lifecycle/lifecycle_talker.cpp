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

#define STRICTLY_DRY 0

class LifecycleTalker : public rclcpp::node::lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp::node::lifecycle::LifecycleNode(node_name, intra_process_comms)
  {
    msg_ = std::make_shared<std_msgs::msg::String>();

#if STRICTLY_DRY
    // Version 1
    pub_ = this->get_communication_interface()->create_publisher<std_msgs::msg::String>("lifecycle_chatter");
    timer_ = this->get_communication_interface()->create_wall_timer(
      1_s, std::bind(&LifecycleTalker::publish, this));
#else
    // Version 2
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter");
    timer_ = this->create_wall_timer(1_s, std::bind(&LifecycleTalker::publish, this));
#endif
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

class LifecycleListener : public rclcpp::node::Node
{
public:
  LifecycleListener(const std::string& node_name)
    : rclcpp::node::Node(node_name)
  {
    sub_data_ = this->create_subscription<std_msgs::msg::String>("lifecycle_chatter", std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));
    sub_notification_ = this->create_subscription<std_msgs::msg::String>("lifecycle_manager__lc_talker", std::bind(&LifecycleListener::notification_callback, this, std::placeholders::_1));
  };

  void data_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "I heard data: [" << msg->data << "]" << std::endl;
  }

  void notification_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "I heard a notification: [" << msg->data << "]" << std::endl;
  }
private:
  std::shared_ptr<rclcpp::subscription::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::subscription::Subscription<std_msgs::msg::String>> sub_notification_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node = std::make_shared<LifecycleTalker>("lc_talker");

#if STRICTLY_DRY
  auto node_name = lc_node->get_base_interface()->get_name();
#else
  auto node_name = lc_node->get_name();
#endif

  rclcpp::lifecycle::LifecycleManager lm;
  lm.add_node_interface(lc_node);


  std::shared_ptr<LifecycleListener> lc_listener = std::make_shared<LifecycleListener>("lc_listener");

  exe.add_node(lc_node->get_communication_interface());
  exe.add_node(lc_listener);

  auto time_out_lambda = []() -> int {
      std::this_thread::sleep_for(std::chrono::seconds(10));
      return 0;
    };

  // configure
  // dummy mockup for now!
  lm.configure(node_name);
  std::shared_future<int> time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  lm.activate(node_name);
  time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  lm.deactivate(node_name);
  time_out = std::async(std::launch::async, time_out_lambda);
  exe.spin_until_future_complete(time_out);

  return 0;
}
