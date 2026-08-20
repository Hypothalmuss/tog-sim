// tog-sim motion server: analytical SCARA IK + Ruckig online trajectory generation streamed at the controller rate.
//
//  action  /togsim/execute_motion   togsim_msgs/action/ExecuteMotion  (sequence of PTP / VIA / TRACK segments)
//  topic   /arm_position_controller/commands (std_msgs/Float64MultiArray) @ control_rate  <- what we publish
//  topic   /joint_states                                                  <- initial state sync
//  topic   /togsim/speed_override (std_msgs/Float32, 0.05..1)             <- scales velocity/accel/jerk online
//  topic   /togsim/motion/busy (std_msgs/Bool)
//  service /togsim/motion/home, /togsim/motion/stop (std_srvs/Trigger)
//
// Why Ruckig streaming instead of a planner: the cell is structured and collision-free by design; what limits the
// cycle rate is latency and stops between segments. Ruckig gives time-optimal jerk-limited motion, allows fly-by
// via points (non-zero target velocity) and re-targeting every tick (moving trays) with no planning latency.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ruckig/ruckig.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <optional>

#include "togsim_motion/scara5_kinematics.hpp"
#include "togsim_msgs/action/execute_motion.hpp"
#include "togsim_msgs/msg/motion_segment.hpp"

using ExecuteMotion = togsim_msgs::action::ExecuteMotion;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMotion>;
using Segment = togsim_msgs::msg::MotionSegment;

namespace togsim {

constexpr size_t DOF = 5;

class MotionServer : public rclcpp::Node {
 public:
  MotionServer() : Node("motion_server"), tfBuffer_(get_clock()), tfListener_(tfBuffer_) {
    rate_ = declare_parameter("control_rate", 500.0);
    joints_ = declare_parameter("joints", std::vector<std::string>{"j1_joint", "j2_joint", "j3_joint", "j4_joint", "tilt_joint"});
    cmdTopic_ = declare_parameter("command_topic", std::string("/arm_position_controller/commands"));
    baseFrame_ = declare_parameter("base_frame", std::string("base_link"));
    auto vmax = declare_parameter("max_velocity", std::vector<double>{10.0, 11.7, 2.3, 49.0, 17.0});
    auto amax = declare_parameter("max_acceleration", std::vector<double>{40, 50, 20, 200, 100});
    auto jmax = declare_parameter("max_jerk", std::vector<double>{400, 500, 300, 2000, 1000});
    auto pmin = declare_parameter("min_position", std::vector<double>{-1.08, -2.57, -0.33, -6.28, -0.785});
    auto pmax = declare_parameter("max_position", std::vector<double>{4.22, 2.57, 0.0, 6.28, 0.785});
    auto home = declare_parameter("home", std::vector<double>{0.0, 1.2, -0.05, 0.0, 0.0});
    kin_.l1 = declare_parameter("l1", 0.40);
    kin_.l2 = declare_parameter("l2", 0.25);
    kin_.z_flange0 = declare_parameter("z_flange0", 0.299);
    kin_.tilt_pivot_drop = declare_parameter("tilt_pivot_drop", 0.045);
    kin_.tool_length = declare_parameter("tool_length", 0.1992);
    kin_.elbow_right = declare_parameter("elbow_right", true);
    settleTol_ = declare_parameter("settle_tolerance", 0.002);
    for (size_t i = 0; i < DOF; ++i) {
      vmax_[i] = vmax[i]; amax_[i] = amax[i]; jmax_[i] = jmax[i]; pmin_[i] = pmin[i]; pmax_[i] = pmax[i]; home_[i] = home[i];
    }
    kin_.j1_min = pmin_[0]; kin_.j1_max = pmax_[0]; kin_.j2_min = pmin_[1]; kin_.j2_max = pmax_[1];
    kin_.j3_min = pmin_[2]; kin_.j3_max = pmax_[2]; kin_.tilt_min = pmin_[4]; kin_.tilt_max = pmax_[4];

    otg_ = std::make_unique<ruckig::Ruckig<DOF>>(1.0 / rate_);
    input_.synchronization = ruckig::Synchronization::Time;
    applyLimits();

    cmdPub_ = create_publisher<std_msgs::msg::Float64MultiArray>(cmdTopic_, rclcpp::QoS(10));
    busyPub_ = create_publisher<std_msgs::msg::Bool>("/togsim/motion/busy", rclcpp::QoS(1).transient_local());
    jsSub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { onJointState(*m); });
    ovSub_ = create_subscription<std_msgs::msg::Float32>("/togsim/speed_override", 10,
        [this](const std_msgs::msg::Float32::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mutex_);
          override_ = std::clamp(static_cast<double>(m->data), 0.05, 1.0);
          applyLimits();
        });
    homeSrv_ = create_service<std_srvs::srv::Trigger>("/togsim/motion/home",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mutex_);
          if (goal_) { res->success = false; res->message = "busy"; return; }
          setJointTarget(home_, {});
          manualMove_ = true;
          res->success = true; res->message = "homing";
        });
    stopSrv_ = create_service<std_srvs::srv::Trigger>("/togsim/motion/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lk(mutex_);
          stopRequested_ = true;
          res->success = true; res->message = "stopping";
        });
    server_ = rclcpp_action::create_server<ExecuteMotion>(this, "/togsim/execute_motion",
        std::bind(&MotionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&MotionServer::handleAccepted, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), std::bind(&MotionServer::tick, this));
    RCLCPP_INFO(get_logger(), "motion_server ready (%.0f Hz, %s)", rate_, cmdTopic_.c_str());
  }

 private:
  // ------------------------------------------------------------------ limits / helpers
  void applyLimits() {
    for (size_t i = 0; i < DOF; ++i) {
      input_.max_velocity[i] = vmax_[i] * override_ * segScale_;
      input_.max_acceleration[i] = amax_[i] * override_ * segScale_;
      input_.max_jerk[i] = jmax_[i] * override_ * segScale_;
    }
  }

  void onJointState(const sensor_msgs::msg::JointState& m) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (synced_) return;
    size_t found = 0;
    std::array<double, DOF> q{};
    for (size_t i = 0; i < DOF; ++i)
      for (size_t k = 0; k < m.name.size(); ++k)
        if (m.name[k] == joints_[i] && k < m.position.size()) { q[i] = m.position[k]; ++found; }
    if (found != DOF) return;
    input_.current_position = q;
    input_.current_velocity.fill(0.0);
    input_.current_acceleration.fill(0.0);
    input_.target_position = q;
    input_.target_velocity.fill(0.0);
    input_.target_acceleration.fill(0.0);
    synced_ = true;
    RCLCPP_INFO(get_logger(), "synced to joint states");
  }

  void setJointTarget(const std::array<double, DOF>& q, const std::array<double, DOF>& v) {
    for (size_t i = 0; i < DOF; ++i) {
      input_.target_position[i] = std::clamp(q[i], pmin_[i], pmax_[i]);
      input_.target_velocity[i] = v[i];
      input_.target_acceleration[i] = 0.0;
    }
  }

  // Convert a segment pose (any frame) to a TcpPose in base_link; returns nullopt if TF is missing.
  std::optional<TcpPose> segmentToTcp(const Segment& s, const std::string& frameOverride = "") {
    geometry_msgs::msg::PoseStamped p = s.pose;
    if (!frameOverride.empty()) p.header.frame_id = frameOverride;
    if (p.header.frame_id.empty()) p.header.frame_id = baseFrame_;
    geometry_msgs::msg::PoseStamped pb = p;
    if (p.header.frame_id != baseFrame_) {
      try {
        auto tf = tfBuffer_.lookupTransform(baseFrame_, p.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(p, pb, tf);
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF %s->%s: %s", baseFrame_.c_str(), p.header.frame_id.c_str(), e.what());
        return std::nullopt;
      }
    }
    TcpPose t;
    t.x = pb.pose.position.x; t.y = pb.pose.position.y; t.z = pb.pose.position.z;
    t.yaw = tf2::getYaw(pb.pose.orientation);
    t.tilt = s.tilt_deg * M_PI / 180.0;
    return t;
  }

  // Joint-space velocity for a Cartesian approach velocity along the tool axis (-z): only J3 moves for a vertical
  // approach, so the via velocity maps to J3 directly (tilted approaches are approximated the same way).
  std::array<double, DOF> viaVelocity(double speedMps, bool descending) const {
    std::array<double, DOF> v{};
    v[2] = descending ? -std::fabs(speedMps) : std::fabs(speedMps);
    return v;
  }

  // ------------------------------------------------------------------ action server
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteMotion::Goal> goal) {
    if (goal->segments.empty()) return rclcpp_action::GoalResponse::REJECT;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!synced_) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>) {
    std::lock_guard<std::mutex> lk(mutex_);
    stopRequested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handleAccepted(const std::shared_ptr<GoalHandle> gh) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (goal_) {  // preempt the running goal: the new one takes over seamlessly from the current state
      auto r = std::make_shared<ExecuteMotion::Result>();
      r->success = false; r->error_code = ExecuteMotion::Result::PREEMPTED; r->message = "preempted";
      goal_->abort(r);
    }
    goal_ = gh;
    segIdx_ = 0;
    segStarted_ = false;
    goalStart_ = now();
    manualMove_ = false;
    stopRequested_ = false;
  }

  bool startSegment(size_t idx) {
    const auto& s = goal_->get_goal()->segments[idx];
    segScale_ = (s.velocity_scale > 0.0f) ? std::clamp(static_cast<double>(s.velocity_scale), 0.05, 1.0) : 1.0;
    applyLimits();
    dwellUntil_.reset();
    if (s.type == Segment::PTP_JOINT) {
      if (s.joint_target.size() != DOF) return fail(ExecuteMotion::Result::LIMITS, "joint_target needs 5 values");
      std::array<double, DOF> q{};
      for (size_t i = 0; i < DOF; ++i) q[i] = s.joint_target[i];
      setJointTarget(q, {});
      return true;
    }
    auto tcp = segmentToTcp(s);
    if (!tcp) return fail(ExecuteMotion::Result::TRACKING_LOST, "pose frame not available");
    auto ik = inverse(*tcp, input_.current_position, kin_, true);
    if (ik.status != IkStatus::Ok) return fail(ExecuteMotion::Result::IK_UNREACHABLE, "IK failed for segment " + std::to_string(idx));
    std::array<double, DOF> v{};
    if (s.type == Segment::VIA_CART && s.via_speed_mps > 0.0f) {
      // descending if the via point is above the next segment's target
      bool descending = true;
      if (idx + 1 < goal_->get_goal()->segments.size()) {
        auto nxt = segmentToTcp(goal_->get_goal()->segments[idx + 1]);
        if (nxt) descending = nxt->z < tcp->z;
      }
      v = viaVelocity(s.via_speed_mps, descending);
    }
    setJointTarget(ik.q, v);
    return true;
  }

  bool fail(uint8_t code, const std::string& msg) {
    auto r = std::make_shared<ExecuteMotion::Result>();
    r->success = false; r->error_code = code; r->message = msg;
    r->duration_s = static_cast<float>((now() - goalStart_).seconds());
    RCLCPP_WARN(get_logger(), "motion failed: %s", msg.c_str());
    goal_->abort(r);
    goal_.reset();
    // brake smoothly
    input_.target_position = input_.current_position;
    input_.target_velocity.fill(0.0);
    return false;
  }

  // ------------------------------------------------------------------ control loop
  void tick() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!synced_) return;

    if (stopRequested_) {
      stopRequested_ = false;
      segScale_ = 1.0; applyLimits();
      input_.target_position = input_.current_position;  // Ruckig brakes to a stop within limits
      input_.target_velocity.fill(0.0);
      input_.target_acceleration.fill(0.0);
      if (goal_) {
        auto r = std::make_shared<ExecuteMotion::Result>();
        r->success = false; r->error_code = ExecuteMotion::Result::PREEMPTED; r->message = "stopped";
        if (goal_->is_canceling()) goal_->canceled(r); else goal_->abort(r);
        goal_.reset();
      }
      manualMove_ = false;
    }

    if (goal_) {
      const auto& segs = goal_->get_goal()->segments;
      if (!segStarted_) {
        if (segIdx_ >= segs.size()) {
          auto r = std::make_shared<ExecuteMotion::Result>();
          r->success = true; r->error_code = ExecuteMotion::Result::SUCCESS; r->message = "done";
          r->duration_s = static_cast<float>((now() - goalStart_).seconds());
          goal_->succeed(r);
          goal_.reset();
          segScale_ = 1.0; applyLimits();
        } else if (startSegment(segIdx_)) {
          segStarted_ = true;
        }
      }
      if (goal_ && segStarted_) {
        const auto& s = segs[segIdx_];
        if (s.type == Segment::TRACK_CART) {  // re-target every tick from the moving frame
          auto tcp = segmentToTcp(s, s.tracked_frame.empty() ? "" : s.tracked_frame);
          if (tcp) {
            auto ik = inverse(*tcp, input_.current_position, kin_, true);
            if (ik.status == IkStatus::Ok) setJointTarget(ik.q, {});
          }
        }
      }
    }

    auto res = otg_->update(input_, output_);
    if (res == ruckig::Result::Error || res == ruckig::Result::ErrorInvalidInput) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ruckig error %d", static_cast<int>(res));
      output_.new_position = input_.current_position;
      output_.new_velocity.fill(0.0);
      output_.new_acceleration.fill(0.0);
    }
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(output_.new_position.begin(), output_.new_position.end());
    cmdPub_->publish(cmd);
    output_.pass_to_input(input_);

    if (goal_ && segStarted_) {
      const auto& s = goal_->get_goal()->segments[segIdx_];
      bool reached = (res == ruckig::Result::Finished);
      if (!reached && s.type == Segment::TRACK_CART) {
        double err = 0.0;
        for (size_t i = 0; i < DOF; ++i) err = std::max(err, std::fabs(input_.current_position[i] - input_.target_position[i]));
        reached = err < settleTol_;
      }
      if (reached) {
        if (s.dwell_s > 0.0f && !dwellUntil_) dwellUntil_ = now() + rclcpp::Duration::from_seconds(s.dwell_s);
        if (!dwellUntil_ || now() >= *dwellUntil_) {
          auto fb = std::make_shared<ExecuteMotion::Feedback>();
          fb->segment_index = static_cast<uint8_t>(segIdx_);
          fb->progress = static_cast<float>(segIdx_ + 1) / goal_->get_goal()->segments.size();
          goal_->publish_feedback(fb);
          ++segIdx_;
          segStarted_ = false;
        }
      }
    }
    bool busy = goal_ != nullptr || (manualMove_ && res != ruckig::Result::Finished);
    if (res == ruckig::Result::Finished) manualMove_ = false;
    if (busy != lastBusy_) { std_msgs::msg::Bool b; b.data = busy; busyPub_->publish(b); lastBusy_ = busy; }
  }

  // ------------------------------------------------------------------ members
  double rate_{500.0};
  std::vector<std::string> joints_;
  std::string cmdTopic_, baseFrame_;
  std::array<double, DOF> vmax_{}, amax_{}, jmax_{}, pmin_{}, pmax_{}, home_{};
  KinematicParams kin_;
  double settleTol_{0.002};
  double override_{1.0}, segScale_{1.0};

  std::unique_ptr<ruckig::Ruckig<DOF>> otg_;
  ruckig::InputParameter<DOF> input_;
  ruckig::OutputParameter<DOF> output_;
  bool synced_{false};
  std::mutex mutex_;

  std::shared_ptr<GoalHandle> goal_;
  size_t segIdx_{0};
  bool segStarted_{false};
  rclcpp::Time goalStart_;
  std::optional<rclcpp::Time> dwellUntil_;
  bool stopRequested_{false}, manualMove_{false}, lastBusy_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmdPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busyPub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jsSub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ovSub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homeSrv_, stopSrv_;
  rclcpp_action::Server<ExecuteMotion>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

}  // namespace togsim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<togsim::MotionServer>();
  rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
