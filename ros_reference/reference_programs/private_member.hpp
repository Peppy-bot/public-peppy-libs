// The private members of ROS classes that the references read, and the wait until a tf
// listener's own thread spins. The access rules do not apply to the names in an explicit
// instantiation, so the explicit instantiation of PrivateMember for a tag defines
// `pointer`, which gives the one member the tag names. PrivateMember defines `pointer` in
// its own namespace, the unnamed namespace of the file that includes this header, where
// the tag declares it, so a file declares each of its tags, and instantiates PrivateMember
// for it, in its unnamed namespace.
#pragma once

#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace ros_reference {

namespace {

template <typename Tag, typename Tag::type member>
struct PrivateMember {
  friend typename Tag::type pointer(Tag) { return member; }
};

struct ListenerExecutor {
  using type = rclcpp::Executor::SharedPtr tf2_ros::TransformListener::*;
  friend type pointer(ListenerExecutor);
};
template struct PrivateMember<ListenerExecutor, &tf2_ros::TransformListener::executor_>;

// Returns once `listener` spins the executor that it spins on a thread of its own, or once
// the default context, which that executor runs on (tf2_ros 0.45.10
// transform_listener.hpp:251), is shut down. The listener's destructor cancels that
// executor and joins the thread (transform_listener.cpp:67-72). The thread's spin() marks
// the executor spinning as it starts, so a cancel before that is lost and the join never
// ends (rclcpp 32.0.3 executor.cpp:468-472, single_threaded_executor.cpp:30-39): on one
// CPU a reference can reach the destructor before the thread first runs. On a shut-down
// context the thread's spin() returns as soon as it starts (:39), so this wait can miss
// the moment the executor spins, and the join ends all the same.
inline void wait_until_spinning(const tf2_ros::TransformListener& listener) {
  const auto& executor = listener.*pointer(ListenerExecutor{});
  if (!executor) {
    throw std::runtime_error("the tf listener spins no executor of its own");
  }
  while (!executor->is_spinning() && rclcpp::ok()) {
    std::this_thread::yield();
  }
}

}  // namespace

}  // namespace ros_reference
