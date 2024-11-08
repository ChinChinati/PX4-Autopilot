// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/RoverAckermannGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/rover_ackermann_guidance_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_RoverAckermannGuidanceStatus_pid_throttle_integral
{
public:
  explicit Init_RoverAckermannGuidanceStatus_pid_throttle_integral(::px4::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::RoverAckermannGuidanceStatus pid_throttle_integral(::px4::msg::RoverAckermannGuidanceStatus::_pid_throttle_integral_type arg)
  {
    msg_.pid_throttle_integral = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_heading_error
{
public:
  explicit Init_RoverAckermannGuidanceStatus_heading_error(::px4::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannGuidanceStatus_pid_throttle_integral heading_error(::px4::msg::RoverAckermannGuidanceStatus::_heading_error_type arg)
  {
    msg_.heading_error = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_pid_throttle_integral(msg_);
  }

private:
  ::px4::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_lookahead_distance
{
public:
  explicit Init_RoverAckermannGuidanceStatus_lookahead_distance(::px4::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannGuidanceStatus_heading_error lookahead_distance(::px4::msg::RoverAckermannGuidanceStatus::_lookahead_distance_type arg)
  {
    msg_.lookahead_distance = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_heading_error(msg_);
  }

private:
  ::px4::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_desired_speed
{
public:
  explicit Init_RoverAckermannGuidanceStatus_desired_speed(::px4::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannGuidanceStatus_lookahead_distance desired_speed(::px4::msg::RoverAckermannGuidanceStatus::_desired_speed_type arg)
  {
    msg_.desired_speed = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_lookahead_distance(msg_);
  }

private:
  ::px4::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_timestamp
{
public:
  Init_RoverAckermannGuidanceStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverAckermannGuidanceStatus_desired_speed timestamp(::px4::msg::RoverAckermannGuidanceStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_desired_speed(msg_);
  }

private:
  ::px4::msg::RoverAckermannGuidanceStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::RoverAckermannGuidanceStatus>()
{
  return px4::msg::builder::Init_RoverAckermannGuidanceStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_
