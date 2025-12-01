// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef HT_MESSAGES__MSG__DETAIL__EXAMPLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HT_MESSAGES__MSG__DETAIL__EXAMPLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ht_messages/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ht_messages/msg/detail/example__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ht_messages
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ht_messages
cdr_serialize(
  const ht_messages::msg::Example & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ht_messages
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ht_messages::msg::Example & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ht_messages
get_serialized_size(
  const ht_messages::msg::Example & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ht_messages
max_serialized_size_Example(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ht_messages

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ht_messages
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ht_messages, msg, Example)();

#ifdef __cplusplus
}
#endif

#endif  // HT_MESSAGES__MSG__DETAIL__EXAMPLE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
