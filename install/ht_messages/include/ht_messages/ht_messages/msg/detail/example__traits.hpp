// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef HT_MESSAGES__MSG__DETAIL__EXAMPLE__TRAITS_HPP_
#define HT_MESSAGES__MSG__DETAIL__EXAMPLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ht_messages/msg/detail/example__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ht_messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const Example & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: foo
  {
    out << "foo: ";
    rosidl_generator_traits::value_to_yaml(msg.foo, out);
    out << ", ";
  }

  // member: bar
  {
    if (msg.bar.size() == 0) {
      out << "bar: []";
    } else {
      out << "bar: [";
      size_t pending_items = msg.bar.size();
      for (auto item : msg.bar) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Example & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: foo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foo: ";
    rosidl_generator_traits::value_to_yaml(msg.foo, out);
    out << "\n";
  }

  // member: bar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bar.size() == 0) {
      out << "bar: []\n";
    } else {
      out << "bar:\n";
      for (auto item : msg.bar) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Example & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ht_messages

namespace rosidl_generator_traits
{

[[deprecated("use ht_messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ht_messages::msg::Example & msg,
  std::ostream & out, size_t indentation = 0)
{
  ht_messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ht_messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const ht_messages::msg::Example & msg)
{
  return ht_messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ht_messages::msg::Example>()
{
  return "ht_messages::msg::Example";
}

template<>
inline const char * name<ht_messages::msg::Example>()
{
  return "ht_messages/msg/Example";
}

template<>
struct has_fixed_size<ht_messages::msg::Example>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ht_messages::msg::Example>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ht_messages::msg::Example>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HT_MESSAGES__MSG__DETAIL__EXAMPLE__TRAITS_HPP_
