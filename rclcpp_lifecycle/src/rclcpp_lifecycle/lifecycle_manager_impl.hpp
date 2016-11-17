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
#include "rclcpp_lifecycle/srv/get_state.hpp"
#include "rclcpp_lifecycle/srv/change_state.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

namespace rclcpp
{
namespace lifecycle
{

using LifecycleInterface = rclcpp::node::lifecycle::LifecycleNode;
using LifecycleInterfacePtr = std::shared_ptr<rclcpp::node::lifecycle::LifecycleNodeInterface>;
using LifecycleInterfaceWeakPtr = std::weak_ptr<rclcpp::node::lifecycle::LifecycleNodeInterface>;

struct NodeStateMachine
{
  LifecycleInterfaceWeakPtr weak_node_handle;
  rcl_state_machine_t state_machine;
  std::map<LifecycleTransitionsT, std::function<bool(void)>> cb_map;
  std::shared_ptr<rclcpp::service::Service<rclcpp_lifecycle::srv::GetState>> notification_server;
};

class LifecycleManager::LifecycleManagerImpl
{
public:
  LifecycleManagerImpl(std::shared_ptr<rclcpp::node::Node> node_base_handle)
  {
    srv_get_state_ = node_base_handle->create_service<rclcpp_lifecycle::srv::GetState>(
        "lifecycle_manager__get_state", std::bind(&LifecycleManagerImpl::on_get_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    srv_change_state_ = node_base_handle->create_service<rclcpp_lifecycle::srv::ChangeState>(
        "lifecycle_manager__change_state", std::bind(&LifecycleManagerImpl::on_change_state, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  };

  ~LifecycleManagerImpl()
  {
    for (auto it=node_handle_map_.begin(); it != node_handle_map_.end(); ++it)
    {
      rcl_state_machine_t* rcl_state_machine = &it->second.state_machine;
      rcl_state_machine_fini(rcl_state_machine);
    }
  }

  void
  on_get_state(const std::shared_ptr<rmw_request_id_t> /*header*/,
      const std::shared_ptr<rclcpp_lifecycle::srv::GetState::Request> req,
      std::shared_ptr<rclcpp_lifecycle::srv::GetState::Response> resp)
  {
    auto node_handle_iter = node_handle_map_.find(req->node_name);
    if (node_handle_iter == node_handle_map_.end())
    {
      resp->current_state = static_cast<uint8_t>(LifecyclePrimaryStatesT::UNKNOWN);
      return;
    }
    resp->current_state = static_cast<uint8_t>(node_handle_iter->second.state_machine.current_state->index);
  }

  void
  on_change_state(const std::shared_ptr<rmw_request_id_t> /*header*/,
      const std::shared_ptr<rclcpp_lifecycle::srv::ChangeState::Request> req,
      std::shared_ptr<rclcpp_lifecycle::srv::ChangeState::Response> resp)
  {
    auto node_handle_iter = node_handle_map_.find(req->node_name);
    if (node_handle_iter == node_handle_map_.end())
    {
      resp->success = false;
      return;
    }
    auto transition = static_cast<LifecycleTransitionsT>(req->transition);
    resp->success = change_state(req->node_name, transition);
  }

  void
  add_node_interface(const std::string & node_name, const LifecycleInterfacePtr & lifecycle_interface)
  {
    rcl_state_machine_t state_machine = rcl_get_zero_initialized_state_machine();
    rcl_state_machine_init(&state_machine, node_name.c_str(), true);
    add_node_interface(node_name, lifecycle_interface, state_machine);
  }

  void
  add_node_interface(const std::string & node_name, const LifecycleInterfacePtr & lifecycle_interface,
    rcl_state_machine_t custom_state_machine)
  {
    NodeStateMachine node_state_machine;
    node_state_machine.weak_node_handle = lifecycle_interface;
    // TODO(karsten1987): Find a way to make this generic to an enduser
    node_state_machine.state_machine = custom_state_machine;

    // register default callbacks
    // maybe optional
    std::function<bool(void)> cb_configuring = std::bind(
        &LifecycleInterface::on_configure, lifecycle_interface);
    std::function<bool(void)> cb_cleaningup = std::bind(
        &LifecycleInterface::on_cleanup, lifecycle_interface);
    std::function<bool(void)> cb_shuttingdown = std::bind(
        &LifecycleInterface::on_shutdown, lifecycle_interface);
    std::function<bool(void)> cb_activating = std::bind(
        &LifecycleInterface::on_activate, lifecycle_interface);
    std::function<bool(void)> cb_deactivating = std::bind(
        &LifecycleInterface::on_deactivate, lifecycle_interface);
    std::function<bool(void)> cb_error = std::bind(
        &LifecycleInterface::on_error, lifecycle_interface);
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

  //template<LifecycleTransitionsT lifecycle_transition>
  bool
  change_state(const std::string & node_name, LifecycleTransitionsT lifecycle_transition)
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
  std::shared_ptr<rclcpp::node::Node> node_base_handle_;
  std::shared_ptr<rclcpp::service::Service<rclcpp_lifecycle::srv::GetState>> srv_get_state_;
  std::shared_ptr<rclcpp::service::Service<rclcpp_lifecycle::srv::ChangeState>> srv_change_state_;
  std::map<std::string, NodeStateMachine> node_handle_map_;
};

}  // namespace lifecycle
}  // namespace rclcpp
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_IMPL_HPP_
