// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ht_messages:msg/Example.idl
// generated code does not contain a copyright notice

#ifndef HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_HPP_
#define HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ht_messages__msg__Example __attribute__((deprecated))
#else
# define DEPRECATED__ht_messages__msg__Example __declspec(deprecated)
#endif

namespace ht_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Example_
{
  using Type = Example_<ContainerAllocator>;

  explicit Example_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->foo = 0;
    }
  }

  explicit Example_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->foo = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _foo_type =
    uint8_t;
  _foo_type foo;
  using _bar_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _bar_type bar;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__foo(
    const uint8_t & _arg)
  {
    this->foo = _arg;
    return *this;
  }
  Type & set__bar(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->bar = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ht_messages::msg::Example_<ContainerAllocator> *;
  using ConstRawPtr =
    const ht_messages::msg::Example_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ht_messages::msg::Example_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ht_messages::msg::Example_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ht_messages::msg::Example_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ht_messages::msg::Example_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ht_messages::msg::Example_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ht_messages::msg::Example_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ht_messages::msg::Example_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ht_messages::msg::Example_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ht_messages__msg__Example
    std::shared_ptr<ht_messages::msg::Example_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ht_messages__msg__Example
    std::shared_ptr<ht_messages::msg::Example_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Example_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->foo != other.foo) {
      return false;
    }
    if (this->bar != other.bar) {
      return false;
    }
    return true;
  }
  bool operator!=(const Example_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Example_

// alias to use template instance with default allocator
using Example =
  ht_messages::msg::Example_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ht_messages

#endif  // HT_MESSAGES__MSG__DETAIL__EXAMPLE__STRUCT_HPP_
