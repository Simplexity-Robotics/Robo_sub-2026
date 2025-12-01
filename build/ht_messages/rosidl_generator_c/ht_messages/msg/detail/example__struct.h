// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_H_
#define HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'bar'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Example in the package ht_messages.
typedef struct ht_messages__msg__Example
{
  std_msgs__msg__Header header;
  uint8_t foo;
  rosidl_runtime_c__float__Sequence bar;
} ht_messages__msg__Example;

// Struct for a sequence of ht_messages__msg__Example.
typedef struct ht_messages__msg__Example__Sequence
{
  ht_messages__msg__Example * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ht_messages__msg__Example__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_H_
