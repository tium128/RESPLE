// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rslidar_msg:msg/RslidarPacket.idl
// generated code does not contain a copyright notice

#ifndef RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__STRUCT_HPP_
#define RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__STRUCT_HPP_

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
# define DEPRECATED__rslidar_msg__msg__RslidarPacket __attribute__((deprecated))
#else
# define DEPRECATED__rslidar_msg__msg__RslidarPacket __declspec(deprecated)
#endif

namespace rslidar_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RslidarPacket_
{
  using Type = RslidarPacket_<ContainerAllocator>;

  explicit RslidarPacket_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_difop = 0;
      this->is_frame_begin = 0;
    }
  }

  explicit RslidarPacket_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_difop = 0;
      this->is_frame_begin = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_difop_type =
    uint8_t;
  _is_difop_type is_difop;
  using _is_frame_begin_type =
    uint8_t;
  _is_frame_begin_type is_frame_begin;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_difop(
    const uint8_t & _arg)
  {
    this->is_difop = _arg;
    return *this;
  }
  Type & set__is_frame_begin(
    const uint8_t & _arg)
  {
    this->is_frame_begin = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rslidar_msg::msg::RslidarPacket_<ContainerAllocator> *;
  using ConstRawPtr =
    const rslidar_msg::msg::RslidarPacket_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rslidar_msg::msg::RslidarPacket_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rslidar_msg::msg::RslidarPacket_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rslidar_msg__msg__RslidarPacket
    std::shared_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rslidar_msg__msg__RslidarPacket
    std::shared_ptr<rslidar_msg::msg::RslidarPacket_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RslidarPacket_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_difop != other.is_difop) {
      return false;
    }
    if (this->is_frame_begin != other.is_frame_begin) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RslidarPacket_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RslidarPacket_

// alias to use template instance with default allocator
using RslidarPacket =
  rslidar_msg::msg::RslidarPacket_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rslidar_msg

#endif  // RSLIDAR_MSG__MSG__DETAIL__RSLIDAR_PACKET__STRUCT_HPP_
