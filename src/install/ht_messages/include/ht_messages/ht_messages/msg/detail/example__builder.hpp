// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef HT_MESSAGES__MSG__DETAIL__EXAMPLE__BUILDER_HPP_
#define HT_MESSAGES__MSG__DETAIL__EXAMPLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ht_messages/msg/detail/example__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ht_messages
{

namespace msg
{

namespace builder
{

class Init_Example_bar
{
public:
  explicit Init_Example_bar(::ht_messages::msg::Example & msg)
  : msg_(msg)
  {}
  ::ht_messages::msg::Example bar(::ht_messages::msg::Example::_bar_type arg)
  {
    msg_.bar = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ht_messages::msg::Example msg_;
};

class Init_Example_foo
{
public:
  explicit Init_Example_foo(::ht_messages::msg::Example & msg)
  : msg_(msg)
  {}
  Init_Example_bar foo(::ht_messages::msg::Example::_foo_type arg)
  {
    msg_.foo = std::move(arg);
    return Init_Example_bar(msg_);
  }

private:
  ::ht_messages::msg::Example msg_;
};

class Init_Example_header
{
public:
  Init_Example_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Example_foo header(::ht_messages::msg::Example::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Example_foo(msg_);
  }

private:
  ::ht_messages::msg::Example msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ht_messages::msg::Example>()
{
  return ht_messages::msg::builder::Init_Example_header();
}

}  // namespace ht_messages

#endif  // HT_MESSAGES__MSG__DETAIL__EXAMPLE__BUILDER_HPP_
