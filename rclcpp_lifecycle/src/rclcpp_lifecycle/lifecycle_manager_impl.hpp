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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_IMPL_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_IMPL_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_manager.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

namespace rclcpp
{
namespace lifecycle
{

using NodeInterface = rclcpp::node::lifecycle::LifecycleNode;
using NodeInterfacePtr = std::shared_ptr<rclcpp::node::lifecycle::LifecycleNodeInterface>;
using NodeInterfaceWeakPtr = std::weak_ptr<rclcpp::node::lifecycle::LifecycleNodeInterface>;

struct NodeStateMachine
{
  NodeInterfaceWeakPtr weak_node_handle;
  rcl_state_machine_t state_machine;
  std::map<LifecycleTransitionsT, std::function<bool(void)>> cb_map;
};

class LIFECYCLE_EXPORT LifecycleManager::LifecycleManagerImpl
{
public:
  LifecycleManagerImpl() = default;
  ~LifecycleManagerImpl() = default;

  void
  add_node_interface(const std::string & node_name, const NodeInterfacePtr & node_interface)
  {
    rcl_state_machine_t state_machine;
    rcl_state_machine_init(&state_machine, node_name.c_str(), true);
    add_node_interface(node_name, node_interface, state_machine);
  }

  void
  add_node_interface(const std::string & node_name, const NodeInterfacePtr & node_interface,
    rcl_state_machine_t custom_state_machine)
  {
    NodeStateMachine node_state_machine;
    node_state_machine.weak_node_handle = node_interface;
    // TODO(karsten1987): Find a way to make this generic to an enduser
    node_state_machine.state_machine = custom_state_machine;

    // register default callbacks
    // maybe optional
    std::function<bool(void)> cb_configuring = std::bind(
      &NodeInterface::on_configure, node_interface);
    std::function<bool(void)> cb_cleaningup = std::bind(
      &NodeInterface::on_cleanup, node_interface);
    std::function<bool(void)> cb_shuttingdown = std::bind(
      &NodeInterface::on_shutdown, node_interface);
    std::function<bool(void)> cb_activating = std::bind(
      &NodeInterface::on_activate, node_interface);
    std::function<bool(void)> cb_deactivating = std::bind(
      &NodeInterface::on_deactivate, node_interface);
    std::function<bool(void)> cb_error = std::bind(
      &NodeInterface::on_error, node_interface);
    node_state_machine.cb_map[LifecycleTransitionsT::CONFIGURING] = cb_configuring;
    node_state_machine.cb_map[LifecycleTransitionsT::CLEANINGUP] = cb_cleaningup;
    node_state_machine.cb_map[LifecycleTransitionsT::SHUTTINGDOWN] = cb_shuttingdown;
    node_state_machine.cb_map[LifecycleTransitionsT::ACTIVATING] = cb_activating;
    node_state_machine.cb_map[LifecycleTransitionsT::DEACTIVATING] = cb_deactivating;
    node_state_machine.cb_map[LifecycleTransitionsT::ERRORPROCESSING] = cb_error;

    // TODO(karsten1987): clarify what do if node already exists;
    node_handle_map_[node_name] = node_state_machine;
  }

  template<LifecycleTransitionsT lifecycle_transition>
  bool
  register_callback(const std::string & node_name, std::function<bool(void)> & cb)
  {
    if (node_name.empty()) {
      return false;
    }

    auto node_handle_iter = node_handle_map_.find(node_name);
    if (node_handle_iter == node_handle_map_.end()) {
      fprintf(stderr, "Node with name %s is not registered\n", node_name.c_str());
    }
    node_handle_iter->second.cb_map[lifecycle_transition] = cb;
    return true;
  }

  template<LifecycleTransitionsT lifecycle_transition>
  bool
  change_state(const std::string & node_name = "")
  {
    if (node_name.empty()) {
      return false;
    }

    auto node_handle_iter = node_handle_map_.find(node_name);
    if (node_handle_iter == node_handle_map_.end()) {
      fprintf(stderr, "%s:%d, Node with name %s is not registered\n",
          __FILE__, __LINE__, node_name.c_str());
      return false;
    }

    auto node_handle = node_handle_iter->second.weak_node_handle.lock();
    if (!node_handle) {
      fprintf(stderr, "%s:%d, Nodehandle is not available. Was it destroyed outside the lifecycle manager?\n",
          __FILE__, __LINE__);
      return false;
    }

    unsigned int transition_index = static_cast<unsigned int>(lifecycle_transition);
    if (!rcl_start_transition_by_index(&node_handle_iter->second.state_machine, transition_index))
    {
      fprintf(stderr, "%s:%d, Unable to start transition %u from current state %s\n",
          __FILE__, __LINE__, transition_index, node_handle_iter->second.state_machine.current_state->label);
      return false;
    }

    // Since we set always set a default callback,
    // we don't have to check for nullptr here
    std::function<bool(void)> callback = node_handle_iter->second.cb_map[lifecycle_transition];
    auto success = callback();

    if (!rcl_finish_transition_by_index(&node_handle_iter->second.state_machine,
          transition_index, success)) {
      fprintf(stderr, "Failed to finish transition %u. Current state is now: %s\n",
          transition_index, node_handle_iter->second.state_machine.current_state->label);
      return false;
    }
    // This true holds in both cases where the actual callback
    // was successful or not, since at this point we have a valid transistion
    // to either a new primary state or error state
    return true;
  }

private:
  std::map<std::string, NodeStateMachine> node_handle_map_;
};

}  // namespace lifecycle
}  // namespace rclcpp
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_IMPL_HPP_
