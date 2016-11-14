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

#if __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rcl/rcl.h"
#include "rosidl_generator_c/message_type_support.h"
#include "rosidl_generator_c/string_functions.h"

//#include "std_msgs/msg/string.h"
#include "rclcpp_lifecycle/msg/transition.h"

#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_lifecycle/transition_map.h"

#include "default_state_machine.h"

//static std_msgs__msg__String msg;
static rclcpp_lifecycle__msg__Transition msg;

const rcl_state_transition_t *
rcl_is_valid_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index)
{
  unsigned int current_index = state_machine->current_state->index;
  const rcl_transition_array_t * valid_transitions = rcl_get_transitions_by_index(
    &state_machine->transition_map, current_index);
  if (valid_transitions == NULL)
  {
    return NULL;
  }
  for (unsigned int i = 0; i < valid_transitions->size; ++i) {
    if (valid_transitions->transitions[i].transition_state.index == transition_index) {
      return &valid_transitions->transitions[i];
    }
  }
  return NULL;
}

const rcl_state_transition_t *
rcl_is_valid_transition_by_label(rcl_state_machine_t * state_machine,
  const char * transition_label)
{
  unsigned int current_index = state_machine->current_state->index;
  const rcl_transition_array_t * valid_transitions = rcl_get_transitions_by_index(
    &state_machine->transition_map, current_index);
  for (unsigned int i = 0; i < valid_transitions->size; ++i) {
    if (valid_transitions->transitions[i].transition_state.label == transition_label) {
      return &valid_transitions->transitions[i];
    }
  }
  return NULL;
}

const rcl_state_transition_t *
rcl_get_registered_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_state_index)
{
  // extensive search approach
  // TODO(karsten1987) can be improved by having a separate array
  // for "registered transition"
  const rcl_transition_map_t * map = &state_machine->transition_map;
  for (unsigned int i = 0; i < map->size; ++i) {
    for (unsigned int j = 0; j < map->transition_arrays[i].size; ++j) {
      if (map->transition_arrays[i].transitions[j].transition_state.index ==
        transition_state_index)
      {
        return &map->transition_arrays[i].transitions[j];
      }
    }
  }
  return NULL;
}

const rcl_state_transition_t *
rcl_get_registered_transition_by_label(rcl_state_machine_t * state_machine,
  const char * transition_state_label)
{
  // extensive search approach
  // TODO(karsten1987) can be improved by having a separate array
  // for "registered transition"
  const rcl_transition_map_t * map = &state_machine->transition_map;
  for (unsigned int i = 0; i < map->size; ++i) {
    for (unsigned int j = 0; j < map->transition_arrays[i].size; ++j) {
      if (strcmp(map->transition_arrays[i].transitions[j].transition_state.label,
        transition_state_label) == 0)
      {
        return &map->transition_arrays[i].transitions[j];
      }
    }
  }
  return NULL;
}

rcl_ret_t
rcl_state_machine_init(rcl_state_machine_t* state_machine, const char* node_name, bool default_states)
{
    {  // initialize node handle for notification
      state_machine->notification_node_handle = rcl_get_zero_initialized_node();
      rcl_node_options_t node_options = rcl_node_get_default_options();
      {
        rcl_ret_t ret = rcl_node_init(&state_machine->notification_node_handle, node_name, &node_options);
        if (ret != RCL_RET_OK)
        {
          fprintf(stderr, "%s:%u, Unable to initialize node handle for state machine\n",
            __FILE__, __LINE__);
          state_machine = NULL;
          return ret;
        }
      }
    }

    {  // initialize publisher
      state_machine->notification_publisher = rcl_get_zero_initialized_publisher();
      const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
        rclcpp_lifecycle, msg, Transition);

      const char* topic_suffix = "lifecycle_manager__";
      if (strlen(node_name)+strlen(topic_suffix) >= 255)
      {
        fprintf(stderr, "%s:%u, Topic name exceeds maximum size of 255\n",
          __FILE__, __LINE__);
        state_machine = NULL;
        return RCL_RET_ERROR;
      }

      char topic_name[255];
      strcpy(topic_name, topic_suffix);
      strcat(topic_name, node_name);
      rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
      rcl_ret_t ret = rcl_publisher_init(&state_machine->notification_publisher, &state_machine->notification_node_handle, ts, topic_name, &publisher_options);
      if (ret != RCL_RET_OK)
      {
        state_machine = NULL;
        return ret;
      }
    }

    if (default_states)
    {
      rcl_get_default_state_machine(state_machine);
    }
    return RCL_RET_OK;
}


void
rcl_register_callback(rcl_state_machine_t * state_machine,
  unsigned int state_index, unsigned int transition_index, bool (* fcn)(void))
{
  rcl_transition_array_t * transitions = rcl_get_transitions_by_index(
    &state_machine->transition_map, state_index);
  for (unsigned int i = 0; i < transitions->size; ++i) {
    if (transitions->transitions[i].transition_state.index == transition_index) {
      transitions->transitions[i].callback = fcn;
    }
  }
}

// maybe change directly the current state here,
// no need to that all the time inside high level language
bool
rcl_start_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index)
{
  const rcl_state_transition_t * transition =
    rcl_is_valid_transition_by_index(state_machine, transition_index);

  // If we have a faulty transition pointer
  if (transition == NULL) {
    fprintf(stderr, "%s:%d, Could not find registered transition %u\n",
        __FILE__, __LINE__, transition_index);
    return false;
  }

  // If we have a transition which is semantically not correct
  // we may have to set the current state to something intermediate
  // or simply ignore it
  if (transition->start != state_machine->current_state) {
    fprintf(stderr, "%s:%d, Wrong transition. State machine is in primary state %s\n",
        __FILE__, __LINE__, state_machine->current_state->label);
    return false;
  }

  // do the initialization
  rclcpp_lifecycle__msg__Transition__init(&msg);
  //rosidl_generator_c__String__assign(&msg.data, "Transition started");
  msg.start_state = state_machine->current_state->index;
  msg.goal_state = transition->transition_state.index;

  if (rcl_publish(&state_machine->notification_publisher, &msg) != RCL_RET_OK){
    fprintf(stderr, "%s:%d, Couldn't publish the notification message.\n",
        __FILE__, __LINE__);
  }
  rclcpp_lifecycle__msg__Transition__fini(&msg);

  // Apply a transition state
  state_machine->current_state = &transition->transition_state;

  return true;
}

bool
rcl_finish_transition_by_index(rcl_state_machine_t * state_machine,
  unsigned int transition_index, bool success)
{
  const rcl_state_transition_t * transition =
    rcl_get_registered_transition_by_index(state_machine, transition_index);

  // If we have a faulty transition pointer
  if (transition == NULL) {
    fprintf(stderr, "%s:%d, Could not find registered transition %u\n",
        __FILE__, __LINE__, transition_index);
    return false;
  }

  // If we have a transition which is semantically not correct
  // we may have to set the current state to something intermediate
  // or simply ignore it
  if (&transition->transition_state != state_machine->current_state) {
    fprintf(stderr, "%s:%d, Wrong transition state. State machine is in primary state %s\n",
        __FILE__, __LINE__, state_machine->current_state->label);
    return false;
  }

  // high level transition(callback) was executed correctly
  if (success == true) {
    rclcpp_lifecycle__msg__Transition__init(&msg);
    msg.start_state = transition->transition_state.index;
    msg.goal_state = transition->goal->index;
    if (rcl_publish(&state_machine->notification_publisher, &msg) != RCL_RET_OK){
      fprintf(stderr, "%s:%d, Couldn't publish the notification message.\n",
          __FILE__, __LINE__);
    }
    rclcpp_lifecycle__msg__Transition__fini(&msg);
    state_machine->current_state = transition->goal;
    return true;
  }

  rclcpp_lifecycle__msg__Transition__init(&msg);
  msg.start_state = transition->transition_state.index;
  msg.goal_state = transition->error->index;
  if (rcl_publish(&state_machine->notification_publisher, &msg) != RCL_RET_OK){
    fprintf(stderr, "%s:%d, Couldn't publish the notification message.\n",
        __FILE__, __LINE__);
  }
  rclcpp_lifecycle__msg__Transition__fini(&msg);
  state_machine->current_state = transition->error;
  return true;
}

#if __cplusplus
}
#endif  // extern "C"
