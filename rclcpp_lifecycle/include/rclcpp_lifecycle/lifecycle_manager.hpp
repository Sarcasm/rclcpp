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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

// forward declaration for c-struct
struct _rcl_state_machine_t;
typedef _rcl_state_machine_t rcl_state_machine_t;

namespace rclcpp
{
namespace lifecycle
{

// *INDENT-OFF*
enum class LifecyclePrimaryStatesT : std::uint8_t
{
  UNCONFIGURED  = 0,
  INACTIVE      = 1,
  ACTIVE        = 2,
  FINALIZED     = 3
};

enum class LifecycleTransitionsT : std::uint8_t
{
  CONFIGURING     = 4,
  CLEANINGUP      = 5,
  SHUTTINGDOWN    = 6,
  ACTIVATING      = 7,
  DEACTIVATING    = 8,
  ERRORPROCESSING = 9
};

class LifecycleManagerInterface
{
  virtual bool configure(const std::string& node_name = "")   = 0;
  virtual bool cleanup(const std::string& node_name = "")     = 0;
  virtual bool shutdown(const std::string& node_name = "")    = 0;
  virtual bool activate(const std::string& node_name = "")    = 0;
  virtual bool deactivate(const std::string& node_name = "")  = 0;
};
// *INDENT-ON*

class LIFECYCLE_EXPORT LifecycleManager : public LifecycleManagerInterface
{
public:
  using NodeInterfacePtr = std::shared_ptr<node::lifecycle::LifecycleNodeInterface>;

  LifecycleManager();
  ~LifecycleManager();

  void
  add_node_interface(const NodeInterfacePtr & node_interface);

  void
  add_node_interface(const NodeInterfacePtr & node_interface,
    rcl_state_machine_t custom_state_machine);

  template<typename T>
  bool
  register_on_configure(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_configure(node_name, cb);
  }

  bool
  register_on_configure(const std::string & node_name, std::function<bool(void)> & fcn);

  bool
  configure(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_cleanup(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_cleanup(node_name, cb);
  }

  bool
  register_on_cleanup(const std::string & node_name, std::function<bool(void)> & fcn);

  bool
  cleanup(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_shutdown(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_shutdown(node_name, cb);
  }

  bool
  register_on_shutdown(const std::string & node_name, std::function<bool(void)> & fcn);

  bool
  shutdown(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_activate(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_activate(node_name, cb);
  }

  bool
  register_on_activate(const std::string & node_name, std::function<bool(void)> & fcn);

  bool
  activate(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_deactivate(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_deactivate(node_name, cb);
  }

  bool
  register_on_deactivate(const std::string & node_name, std::function<bool(void)> & fcn);

  bool
  deactivate(const std::string & node_name = "");

private:
  class LIFECYCLE_EXPORT LifecycleManagerImpl;
  std::unique_ptr<LifecycleManagerImpl> impl_;
};

}  // namespace lifecycle
}  // namespace rclcpp
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
