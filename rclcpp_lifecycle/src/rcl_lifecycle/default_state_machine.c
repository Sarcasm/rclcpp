// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rcl/rcl.h"
#include "rosidl_generator_c/message_type_support.h"
#include "rosidl_generator_c/string_functions.h"
#include "std_msgs/msg/string.h"

#include "rcl_lifecycle/data_types.h"
#include "rcl_lifecycle/states.h"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_lifecycle/transition_map.h"

#if __cplusplus
extern "C"
{
#endif

const rcl_state_t rcl_state_unconfigured     = {"unconfigured", 0};
const rcl_state_t rcl_state_inactive         = {"inactive", 1};
const rcl_state_t rcl_state_active           = {"active", 2};
const rcl_state_t rcl_state_finalized        = {"finalized", 3};
const rcl_state_t rcl_state_configuring      = {"configuring", 4};
const rcl_state_t rcl_state_cleaningup       = {"cleaningup", 5};
const rcl_state_t rcl_state_shuttingdown     = {"shuttingdown", 6};
const rcl_state_t rcl_state_activating       = {"activating", 7};
const rcl_state_t rcl_state_deactivating     = {"deactivating", 8};
const rcl_state_t rcl_state_errorprocessing  = {"errorprocessing", 9};

rcl_state_t
rcl_create_state(unsigned int index, char * label)
{
  rcl_state_t ret_state = {.index = index, .label = label};
  return ret_state;
}

rcl_state_transition_t
rcl_create_state_transition(unsigned int index, const char * label)
{
  rcl_state_transition_t ret_transition = {{.index = index, .label = label},
    NULL, NULL, NULL, &rcl_state_errorprocessing};
  return ret_transition;
}

// default implementation as despicted on
// design.ros2.org
rcl_state_machine_t
rcl_get_default_state_machine(const char* node_name)
{
  rcl_state_machine_t state_machine;

  // initialize node handle for notification
  state_machine.notification_node_handle = rcl_get_zero_initialized_node();
  rcl_node_options_t node_options = rcl_node_get_default_options();
  if (rcl_node_init(&state_machine.notification_node_handle, node_name, &node_options) != RCL_RET_OK)
  {
    state_machine.current_state = NULL;
    return state_machine;
  }

  // initialize publisher
  state_machine.notification_publisher = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
      std_msgs, msg, String);
  char topic_name[100];
  strcpy(topic_name,"lifecycle_manager__");
  strcat(topic_name, node_name);
  rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
  if (rcl_publisher_init(&state_machine.notification_publisher, &state_machine.notification_node_handle, ts,
      topic_name, &publisher_options) != RCL_RET_OK)
  {
    state_machine.current_state = NULL;
    return state_machine;
  }

  state_machine.transition_map.primary_states = NULL;
  state_machine.transition_map.transition_arrays = NULL;
  state_machine.transition_map.size = 0;

  rcl_state_transition_t rcl_transition_configuring = rcl_create_state_transition(
    rcl_state_configuring.index, rcl_state_configuring.label);
  rcl_state_transition_t rcl_transition_shuttingdown = rcl_create_state_transition(
    rcl_state_shuttingdown.index, rcl_state_shuttingdown.label);
  rcl_state_transition_t rcl_transition_cleaningup = rcl_create_state_transition(
    rcl_state_cleaningup.index, rcl_state_cleaningup.label);
  rcl_state_transition_t rcl_transition_activating = rcl_create_state_transition(
    rcl_state_activating.index, rcl_state_activating.label);
  rcl_state_transition_t rcl_transition_deactivating = rcl_create_state_transition(
    rcl_state_deactivating.index, rcl_state_deactivating.label);

  rcl_register_primary_state(&state_machine.transition_map, rcl_state_unconfigured);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_inactive);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_active);
  rcl_register_primary_state(&state_machine.transition_map, rcl_state_finalized);

  // add transitions to map
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_unconfigured, rcl_state_inactive, rcl_transition_configuring);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_inactive, rcl_state_unconfigured, rcl_transition_cleaningup);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_unconfigured, rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_inactive, rcl_state_finalized, rcl_transition_shuttingdown);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_inactive, rcl_state_active, rcl_transition_activating);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_active, rcl_state_inactive, rcl_transition_deactivating);
  rcl_register_transition_by_state(&state_machine.transition_map,
    rcl_state_active, rcl_state_finalized, rcl_transition_shuttingdown);

  // set to first entry
  state_machine.current_state = &state_machine.transition_map.primary_states[0];
  return state_machine;
}

#if __cplusplus
}
#endif  // extern "C"
