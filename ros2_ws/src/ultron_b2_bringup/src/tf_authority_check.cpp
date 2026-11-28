// tf_authority_check: fail when any TF child frame has more than one authority.
//
// Subscribes /tf and /tf_static and records, per child frame, the parent frames and the
// DDS publisher GIDs (rclcpp::MessageInfo; rclpy on Humble does not expose them).
// Violations (all are ERROR):
//   MULTIPLE_PUBLISHERS      child published by >1 publisher on /tf or on /tf_static
//   STATIC_DYNAMIC_CONFLICT  child appears on both /tf and /tf_static
//   MULTIPLE_PARENTS         child attached to more than one parent
//   GROUND_TRUTH_FRAME       a forbidden (ground-truth) frame appears on TF
// On a violation it publishes a DiagnosticArray (relative topic "diagnostics") and, when
// fail_on_violation is true, exits with code 1 so the bringup launch shuts the stack down.
// Limitation: a node that restarts and republishes a static transform gets a new GID and is
// reported as a second publisher (fail-closed).
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{
using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

struct FrameRecord
{
  std::set<std::string> parents;
  std::set<Gid> dynamic_publishers;
  std::set<Gid> static_publishers;
  size_t unknown_publisher_messages = 0;
};
}  // namespace

class TfAuthorityCheck : public rclcpp::Node
{
public:
  TfAuthorityCheck()
  : Node("tf_authority_check")
  {
    forbidden_ = declare_parameter<std::vector<std::string>>(
      "forbidden_frames", {"ultron_sim_world", "ground_truth", "gt", "world_gt"});
    fail_on_violation_ = declare_parameter<bool>("fail_on_violation", true);
    duration_s_ = declare_parameter<double>("duration_s", 0.0);
    const double period = declare_parameter<double>("report_period_s", 1.0);

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    auto dyn_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable();
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", dyn_qos,
      [this](std::shared_ptr<const tf2_msgs::msg::TFMessage> msg, const rclcpp::MessageInfo & info) {
        on_tf(*msg, info, false);
      });
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", static_qos,
      [this](std::shared_ptr<const tf2_msgs::msg::TFMessage> msg, const rclcpp::MessageInfo & info) {
        on_tf(*msg, info, true);
      });
    start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.05, period)), [this]() {report();});
    RCLCPP_INFO(get_logger(), "watching /tf and /tf_static (fail_on_violation=%s)",
      fail_on_violation_ ? "true" : "false");
  }

  int exit_code() const {return exit_code_;}

private:
  void on_tf(const tf2_msgs::msg::TFMessage & msg, const rclcpp::MessageInfo & info, bool is_static)
  {
    Gid gid{};
    const auto & raw = info.get_rmw_message_info().publisher_gid.data;
    std::copy(std::begin(raw), std::begin(raw) + gid.size(), gid.begin());
    const bool known = std::any_of(gid.begin(), gid.end(), [](uint8_t b) {return b != 0;});
    for (const auto & t : msg.transforms) {
      const std::string child = strip(t.child_frame_id);
      const std::string parent = strip(t.header.frame_id);
      for (const auto & f : {child, parent}) {
        if (std::find(forbidden_.begin(), forbidden_.end(), f) != forbidden_.end()) {
          violation("GROUND_TRUTH_FRAME", child, "forbidden frame '" + f + "' on " + topic(is_static));
        }
      }
      auto & rec = frames_[child];
      rec.parents.insert(parent);
      if (rec.parents.size() > 1) {
        violation("MULTIPLE_PARENTS", child, "parents: " + join(rec.parents));
      }
      if (!known) {
        ++rec.unknown_publisher_messages;
        continue;
      }
      auto & pubs = is_static ? rec.static_publishers : rec.dynamic_publishers;
      pubs.insert(gid);
      if (pubs.size() > 1) {
        violation("MULTIPLE_PUBLISHERS", child,
          std::to_string(pubs.size()) + " publishers on " + topic(is_static));
      }
      if (!rec.static_publishers.empty() && !rec.dynamic_publishers.empty()) {
        violation("STATIC_DYNAMIC_CONFLICT", child, "published on both /tf and /tf_static");
      }
    }
  }

  static std::string strip(const std::string & frame)
  {
    return (!frame.empty() && frame[0] == '/') ? frame.substr(1) : frame;
  }
  static std::string topic(bool is_static) {return is_static ? "/tf_static" : "/tf";}
  static std::string join(const std::set<std::string> & items)
  {
    std::string out;
    for (const auto & s : items) {out += (out.empty() ? "" : ",") + s;}
    return out;
  }

  void violation(const std::string & kind, const std::string & child, const std::string & detail)
  {
    const std::string key = kind + ":" + child;
    if (!reported_.insert(key).second) {
      return;
    }
    RCLCPP_ERROR(get_logger(), "TF_AUTHORITY_VIOLATION %s child=%s %s", kind.c_str(), child.c_str(),
      detail.c_str());
    violations_.push_back(key + " " + detail);
    publish(diagnostic_msgs::msg::DiagnosticStatus::ERROR, key + " " + detail);
    if (fail_on_violation_) {
      exit_code_ = 1;
      rclcpp::shutdown();
    }
  }

  void publish(uint8_t level, const std::string & message)
  {
    if (!rclcpp::ok()) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "ultron/tf_authority_check";
    status.hardware_id = "none";
    status.message = message;
    for (const auto & [child, rec] : frames_) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = child;
      kv.value = "parents=" + join(rec.parents) + " dynamic_publishers=" +
        std::to_string(rec.dynamic_publishers.size()) + " static_publishers=" +
        std::to_string(rec.static_publishers.size()) + " unknown_publisher_msgs=" +
        std::to_string(rec.unknown_publisher_messages);
      status.values.push_back(kv);
    }
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  void report()
  {
    if (!violations_.empty()) {
      publish(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::to_string(violations_.size()) + " violation(s); first: " + violations_.front());
    } else {
      publish(diagnostic_msgs::msg::DiagnosticStatus::OK,
        std::to_string(frames_.size()) + " child frame(s), single authority each");
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    if (duration_s_ > 0.0 && elapsed >= duration_s_) {
      RCLCPP_INFO(get_logger(), "duration elapsed: %zu frame(s), %zu violation(s)", frames_.size(),
        violations_.size());
      for (const auto & [child, rec] : frames_) {
        RCLCPP_INFO(get_logger(), "frame %s parents=%s dynamic_publishers=%zu static_publishers=%zu",
          child.c_str(), join(rec.parents).c_str(), rec.dynamic_publishers.size(),
          rec.static_publishers.size());
      }
      exit_code_ = violations_.empty() ? 0 : 1;
      rclcpp::shutdown();
    }
  }

  std::vector<std::string> forbidden_;
  bool fail_on_violation_{true};
  double duration_s_{0.0};
  int exit_code_{0};
  std::chrono::steady_clock::time_point start_;
  std::map<std::string, FrameRecord> frames_;
  std::set<std::string> reported_;
  std::vector<std::string> violations_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  {
    auto node = std::make_shared<TfAuthorityCheck>();
    rclcpp::spin(node);
    code = node->exit_code();
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  std::fflush(stdout);
  return code;
}
