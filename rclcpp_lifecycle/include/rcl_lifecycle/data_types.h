#ifndef RCL_LIFECYCLE__DATA_TYPES_H_
#define RCL_LIFECYCLE__DATA_TYPES_H_

#include <rcl/rcl.h>
#include "rcl_lifecycle/visibility_control.h"

#if __cplusplus
extern "C"
{
#endif

/**
 * @brief simple definition of a state
 * @param state: integer giving the state
 * @param label: label for easy indexing
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_t
{
  const char * label;
  unsigned int index;
} rcl_state_t;

/**
 * @brief transition definition
 * @param start: rcl_state_t as a start state
 * @param goal: rcl_state_t as a goal state
 * TODO: Maybe specify callback pointer here
 * and call on_* functions directly
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_transition_t
{
  rcl_state_t transition_state;
  void * callback;
  const rcl_state_t * start;
  const rcl_state_t * goal;
  const rcl_state_t * error;
} rcl_state_transition_t;

/**
 * @brief All transitions which are
 * valid associations for a primary state.
 * One array belongs to one primary state
 * within the map.
 */
typedef struct LIFECYCLE_EXPORT _rcl_transition_array_t
{
  rcl_state_transition_t * transitions;
  unsigned int size;
} rcl_transition_array_t;

/**
 * @brief stores an array of transitions
 * index by a start state
 */
typedef struct LIFECYCLE_EXPORT _rcl_transition_map_t
{
  // associative array between primary state
  // and their respective transitions
  // 1 ps -> 1 transition_array
  rcl_state_t * primary_states;
  rcl_transition_array_t * transition_arrays;
  unsigned int size;
} rcl_transition_map_t;

/**
 * @brief: statemachine object holding
 * a variable state object as current state
 * of the complete machine.
 * @param transition_map: a map object of all
 * possible transitions registered with this
 * state machine.
 */
typedef struct LIFECYCLE_EXPORT _rcl_state_machine_t
{
  const rcl_state_t * current_state;
  rcl_transition_map_t transition_map;
  // TODO(karsten1987): Maybe encapsulate this into
  // a ROS communication struct...
  rcl_node_t notification_node_handle;
  rcl_publisher_t notification_publisher;
  rcl_service_t notification_service;
} rcl_state_machine_t;

#if __cplusplus
}
#endif

#endif  // RCL_LIFECYCLE__DATA_TYPES_H_
