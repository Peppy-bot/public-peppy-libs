#include "transform_lookup.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

namespace ros_reference {

namespace {

// The authority every transform is sent under: the scenarios have one publisher.
const std::string publisher = "transform_lookup";

constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;

// The transform of a step, at the step's stamp, or at none for a static transform.
geometry_msgs::msg::TransformStamped transform_of(const nlohmann::json& step) {
  geometry_msgs::msg::TransformStamped transform;
  const auto stamp = step.contains("stamp") ? step.at("stamp").get<std::int64_t>() : 0;
  transform.header.stamp.sec = static_cast<std::int32_t>(stamp / nanoseconds_per_second);
  transform.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % nanoseconds_per_second);
  transform.header.frame_id = step.at("parent").get<std::string>();
  transform.child_frame_id = step.at("child").get<std::string>();
  const auto position = step.at("position").get<std::vector<double>>();
  const auto orientation = step.at("orientation").get<std::vector<double>>();
  if (position.size() != 3 || orientation.size() != 4) {
    throw std::runtime_error("a transform holds a position of 3 and an orientation of 4");
  }
  transform.transform.translation.x = position[0];
  transform.transform.translation.y = position[1];
  transform.transform.translation.z = position[2];
  transform.transform.rotation.x = orientation[0];
  transform.transform.rotation.y = orientation[1];
  transform.transform.rotation.z = orientation[2];
  transform.transform.rotation.w = orientation[3];
  return transform;
}

// The time a step names in nanoseconds, or tf2's time zero, the latest time, for null.
tf2::TimePoint time_of(const nlohmann::json& time) {
  return time.is_null() ? tf2::TimePointZero
                        : tf2::TimePoint(std::chrono::nanoseconds(time.get<std::int64_t>()));
}

// The answer of `look`, a lookup of tf2's: the source's transform in the target and the time
// it holds at, null for the time zero, or the error, by the exception tf2 throws. tf2 throws
// a LookupException both for a frame it does not hold and for a loop; the message tells
// them apart (buffer_core.cpp:149-156, 398-406).
template <typename Look>
nlohmann::json answer_of(const Look& look) {
  try {
    const geometry_msgs::msg::TransformStamped found = look();
    const auto& t = found.transform;
    const std::int64_t stamp =
        std::int64_t{found.header.stamp.sec} * nanoseconds_per_second + found.header.stamp.nanosec;
    return {{"position", {t.translation.x, t.translation.y, t.translation.z}},
            {"orientation", {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w}},
            {"time", stamp == 0 ? nlohmann::json(nullptr) : nlohmann::json(stamp)}};
  } catch (const tf2::ConnectivityException&) {
    return {{"error", "connectivity"}};
  } catch (const tf2::ForwardExtrapolationException&) {
    return {{"error", "forward_extrapolation"}};
  } catch (const tf2::BackwardExtrapolationException&) {
    return {{"error", "backward_extrapolation"}};
  } catch (const tf2::NoDataForExtrapolationException&) {
    return {{"error", "no_data_for_extrapolation"}};
  } catch (const tf2::InvalidArgumentException&) {
    return {{"error", "invalid_argument"}};
  } catch (const tf2::LookupException& error) {
    const bool loop = std::string(error.what()).find("contains a loop") != std::string::npos;
    return {{"error", loop ? "loop" : "unknown_frame"}};
  }
}

// The answer to a lookup of the source in the target at the step's time.
nlohmann::json lookup(const tf2::BufferCore& buffer, const nlohmann::json& step) {
  return answer_of([&] {
    return buffer.lookupTransform(step.at("target").get<std::string>(),
                                  step.at("source").get<std::string>(), time_of(step.at("time")));
  });
}

// The answer to a lookup through a fixed frame, tf2's advanced lookupTransform: the source at
// source_time in the fixed frame, then the fixed frame in the target at target_time
// (buffer_core.cpp:750-781, 880-898).
nlohmann::json lookup_through(const tf2::BufferCore& buffer, const nlohmann::json& step) {
  return answer_of([&] {
    return buffer.lookupTransform(
        step.at("target").get<std::string>(), time_of(step.at("target_time")),
        step.at("source").get<std::string>(), time_of(step.at("source_time")),
        step.at("fixed").get<std::string>());
  });
}

}  // namespace

nlohmann::json transform_lookup(const nlohmann::json& scenario, Inputs&) {
  tf2::BufferCore buffer(
      tf2::Duration(std::chrono::nanoseconds(scenario.at("cache_window_ns").get<std::int64_t>())));
  // The static transforms sent so far, as tf2_ros's static broadcaster holds them: a transform
  // replaces the one of its child frame, and each send carries them all
  // (static_transform_broadcaster.cpp:52-71).
  std::vector<geometry_msgs::msg::TransformStamped> static_transforms_sent;
  auto answers = nlohmann::json::array();
  for (const auto& step : scenario.at("steps")) {
    const auto kind = step.at("step").get<std::string>();
    if (kind == "moving") {
      answers.push_back({{"taken", buffer.setTransform(transform_of(step), publisher, false)}});
    } else if (kind == "static") {
      const auto sent = transform_of(step);
      bool replaced = false;
      for (auto& held : static_transforms_sent) {
        if (held.child_frame_id == sent.child_frame_id) {
          held = sent;
          replaced = true;
        }
      }
      if (!replaced) {
        static_transforms_sent.push_back(sent);
      }
      bool taken = false;
      for (const auto& held : static_transforms_sent) {
        const bool took = buffer.setTransform(held, publisher, true);
        if (held.child_frame_id == sent.child_frame_id) {
          taken = took;
        }
      }
      answers.push_back({{"taken", taken}});
    } else if (kind == "lookup") {
      answers.push_back(lookup(buffer, step));
    } else if (kind == "lookup_through") {
      answers.push_back(lookup_through(buffer, step));
    } else {
      throw std::runtime_error("no step is named " + kind);
    }
  }
  return {{"kind", "transform_lookup"}, {"steps", answers}};
}

}  // namespace ros_reference
