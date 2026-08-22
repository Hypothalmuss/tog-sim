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
#include <algorithm>
#include <cmath>
#include <cstdio>
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
    trackSettleTol_ = declare_parameter("track_settle_tolerance", 0.004);  // Cartesian, m
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
    // Tick on the node clock (sim time): with a wall timer the trajectory outran the physics whenever the simulator
    // ran below real time (RTF 0.6-0.9 with the vision stack), and the jam guard tripped on the resulting 3-5 cm lag.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), std::bind(&MotionServer::tick, this));
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
    size_t found = 0;
    std::array<double, DOF> q{}, v{};
    bool haveVel = m.velocity.size() == m.position.size();
    for (size_t i = 0; i < DOF; ++i)
      for (size_t k = 0; k < m.name.size(); ++k)
        if (m.name[k] == joints_[i] && k < m.position.size()) {
          q[i] = m.position[k];
          if (haveVel) v[i] = m.velocity[k];
          ++found;
        }
    if (found != DOF) return;
    actual_ = q;
    if (haveVel) actualVel_ = v;
    haveActualVel_ = haveVel;
    haveActual_ = true;
    if (synced_) return;
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
    if (p.header.frame_id == "tcp") {  // relative to where the tool is right now (e.g. "lift 60 mm straight up")
      TcpPose cur = forward(input_.current_position, kin_);
      TcpPose t;
      t.x = cur.x + p.pose.position.x; t.y = cur.y + p.pose.position.y; t.z = cur.z + p.pose.position.z;
      // a relative lift/descent cannot leave the vertical stroke: clamp to the TCP heights J3 can reach (at tilt 0)
      const double zTcpMin = pmin_[2] + kin_.z_flange0 - kin_.tilt_pivot_drop - kin_.tool_length;
      const double zTcpMax = pmax_[2] + kin_.z_flange0 - kin_.tilt_pivot_drop - kin_.tool_length;
      t.z = std::clamp(t.z, zTcpMin, zTcpMax);
      t.yaw = cur.yaw + tf2::getYaw(p.pose.orientation);
      t.tilt = s.tilt_deg * M_PI / 180.0;
      return t;
    }
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
    trackPrevValid_ = false;
    trackVel_.fill(0.0);
    trackErr_ = 1.0;
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
    if (ik.status != IkStatus::Ok) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "IK failed for segment %zu: %s at (%.3f, %.3f, %.3f) yaw %.1f deg tilt %.1f deg", idx,
                    ik.status == IkStatus::OutOfLimits ? "out of joint limits" : "unreachable", tcp->x, tcp->y, tcp->z,
                    tcp->yaw * 180.0 / M_PI, tcp->tilt * 180.0 / M_PI);
      return fail(ExecuteMotion::Result::IK_UNREACHABLE, buf);
    }
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
            if (ik.status == IkStatus::Ok) {
              // Frame velocity: low-pass filtered Cartesian displacement between TF updates (constant on a belt), then
              // the joint-space feed-forward from a second IK solve a little ahead along that velocity. A zero target
              // velocity made the generator decelerate towards every new target and trail the product by centimetres.
              const auto tnow = now();
              const std::array<double, 3> pnow{tcp->x, tcp->y, tcp->z};
              if (!trackPrevValid_) {
                trackPrevP_ = pnow; trackPrevTime_ = tnow; trackPrevValid_ = true; trackVelCart_.fill(0.0);
              } else {
                bool moved = false;
                for (size_t i = 0; i < 3; ++i) moved |= std::fabs(pnow[i] - trackPrevP_[i]) > 1e-5;
                const double dt = (tnow - trackPrevTime_).seconds();
                if (moved && dt > 1e-3) {
                  for (size_t i = 0; i < 3; ++i) {
                    const double sample = std::clamp((pnow[i] - trackPrevP_[i]) / dt, -1.0, 1.0);
                    trackVelCart_[i] = 0.85 * trackVelCart_[i] + 0.15 * sample;
                  }
                  trackPrevP_ = pnow; trackPrevTime_ = tnow;
                }
              }
              trackVel_.fill(0.0);
              constexpr double kDelta = 0.02;  // s
              TcpPose ahead = *tcp;
              ahead.x += trackVelCart_[0] * kDelta; ahead.y += trackVelCart_[1] * kDelta; ahead.z += trackVelCart_[2] * kDelta;
              auto ik2 = inverse(ahead, ik.q, kin_, true);
              if (ik2.status == IkStatus::Ok)
                for (size_t i = 0; i < DOF; ++i) trackVel_[i] = std::clamp((ik2.q[i] - ik.q[i]) / kDelta, -0.5 * vmax_[i], 0.5 * vmax_[i]);
              setJointTarget(ik.q, trackVel_);
              const TcpPose cur = forward(input_.current_position, kin_);
              trackErr_ = std::sqrt(std::pow(cur.x - tcp->x, 2) + std::pow(cur.y - tcp->y, 2) + std::pow(cur.z - tcp->z, 2));
              RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500, "track seg %zu: err %.1f mm, v_cart [%.3f %.3f %.3f]", segIdx_,
                                    trackErr_ * 1e3, trackVelCart_[0], trackVelCart_[1], trackVelCart_[2]);
            } else {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "track seg %zu: target unreachable (%.3f, %.3f)", segIdx_, tcp->x, tcp->y);
            }
          }
        }
      }
    }

    // Jam guard: the arm is position-tracked, so a persistent error between command and measurement means it is
    // physically blocked (e.g. pressing a product into a tray). Abort the goal and brake instead of thrashing.
    // Signature of a real jam: the vertical command keeps moving while the measured joint does not (tracking lag alone
    // - transport latency, low RTF, fast descents - shows a moving joint and must not trip the guard).
    if (goal_ && haveActual_) {
      const double err = std::fabs(actual_[2] - input_.current_position[2]);
      const double tol = jamTolerance_ + 0.03 * std::fabs(input_.current_velocity[2]);
      // readings outside the joint limits are solver glitches (e.g. at weld creation), not jams: ignore them
      bool plausible = true;
      for (size_t i = 0; i < DOF; ++i)
        if (actual_[i] < pmin_[i] - 0.02 || actual_[i] > pmax_[i] + 0.02) plausible = false;
      const bool commanded = std::fabs(input_.current_velocity[2]) > 0.03;
      const bool stalled = !haveActualVel_ || std::fabs(actualVel_[2]) < 0.01;
      jamTicks_ = (plausible && err > tol && commanded && stalled) ? jamTicks_ + 1 : 0;
      if (jamTicks_ > static_cast<int>(0.15 * rate_)) {
        jamTicks_ = 0;
        // resync the generator to where the arm really is, then fail the goal
        input_.current_position = actual_;
        input_.current_velocity.fill(0.0);
        input_.current_acceleration.fill(0.0);
        fail(ExecuteMotion::Result::CONTROLLER_ERROR, "jam detected: vertical tracking error " + std::to_string(err) + " m");
      }
    }

    auto res = otg_->update(input_, output_);
    if (res == ruckig::Result::Error || res == ruckig::Result::ErrorInvalidInput) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ruckig error %d", static_cast<int>(res));
      output_.new_position = input_.current_position;
      output_.new_velocity.fill(0.0);
      output_.new_acceleration.fill(0.0);
    }
    // never publish a command outside the joint limits (the controller would apply it verbatim)
    for (size_t i = 0; i < DOF; ++i) output_.new_position[i] = std::clamp(output_.new_position[i], pmin_[i], pmax_[i]);
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(output_.new_position.begin(), output_.new_position.end());
    cmdPub_->publish(cmd);
    output_.pass_to_input(input_);

    if (goal_ && segStarted_) {
      const auto& s = goal_->get_goal()->segments[segIdx_];
      bool reached = (res == ruckig::Result::Finished);
      if (!reached && s.type == Segment::TRACK_CART)  // a tracked target is never "finished": settle within tolerance
        reached = trackPrevValid_ && trackErr_ < trackSettleTol_;
      if (dwellUntil_) reached = true;  // once dwelling, the dwell expiry alone ends the segment (tracking continues)
      if (reached) {
        if (s.dwell_s > 0.0f && !dwellUntil_) {
          dwellUntil_ = now() + rclcpp::Duration::from_seconds(s.dwell_s);
          // "reached, dwelling": clients use this to fire the vacuum / release exactly at contact (progress = idx + 0.5)
          auto fb = std::make_shared<ExecuteMotion::Feedback>();
          fb->segment_index = static_cast<uint8_t>(segIdx_);
          fb->progress = (static_cast<float>(segIdx_) + 0.5f) / goal_->get_goal()->segments.size();
          goal_->publish_feedback(fb);
        }
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
  double trackSettleTol_{0.006};
  std::array<double, DOF> trackVel_{}, actualVel_{};
  bool haveActualVel_{false};
  std::array<double, 3> trackPrevP_{}, trackVelCart_{};
  rclcpp::Time trackPrevTime_;
  bool trackPrevValid_{false};
  double trackErr_{1.0};
  double override_{1.0}, segScale_{1.0};

  std::unique_ptr<ruckig::Ruckig<DOF>> otg_;
  ruckig::InputParameter<DOF> input_;
  ruckig::OutputParameter<DOF> output_;
  bool synced_{false};
  std::array<double, DOF> actual_{};
  bool haveActual_{false};
  int jamTicks_{0};
  double jamTolerance_{0.010};
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
