// tog-sim ConveyorSystem — a deterministic conveyor belt for Gazebo Fortress.
//
// Attach to a STATIC belt model. Every dynamic model whose origin lies inside the belt footprint and within
// <capture_height> above the belt surface is carried along <direction> at the commanded speed (x/y velocity is
// imposed, the vertical velocity is passed through so objects can still settle or fall). Models currently held by a
// DetachableJoint (e.g. the vacuum gripper) are ignored. The commanded speed is ramped with <max_acceleration>.
//
// Why not TrackController: on Fortress/DART 6.12 the contact-surface-velocity mechanism does not move objects
// reliably (verified with the stock conveyor.sdf example); a kinematic belt link is not supported by DART here either.
//
// SDF parameters (all optional unless noted):
//   <belt_link>         link of this model whose frame defines the belt (required)
//   <surface_size>      "length width" of the footprint in the belt link frame (default 2.0 0.5)
//   <surface_offset>    "x y z" of the footprint centre / top surface in the belt link frame (default 0 0 0)
//   <direction>         travel direction in the belt link frame (default 1 0 0)
//   <capture_height>    band above the surface in which models are carried (default 0.08)
//   <initial_velocity>  m/s (default 0)
//   <max_acceleration>  m/s^2 ramp (default 2.0)
//   <cmd_topic>         ignition.msgs.Double (default /model/<model>/conveyor/cmd_vel)
//   <state_topic>       ignition.msgs.Double, actual belt speed @ 50 Hz (default /model/<model>/conveyor/state)
//   <model_prefix>      only carry models whose name starts with this (default "" = any dynamic model)
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>

namespace togsim {

namespace gz = ignition::gazebo;

class ConveyorSystem : public gz::System, public gz::ISystemConfigure, public gz::ISystemPreUpdate {
 public:
  void Configure(const gz::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::EntityComponentManager& ecm, gz::EventManager&) override {
    model_ = gz::Model(entity);
    const std::string modelName = model_.Name(ecm);
    if (!sdf->HasElement("belt_link")) {
      ignerr << "[ConveyorSystem] <belt_link> is required (model " << modelName << ")\n";
      return;
    }
    beltLinkName_ = sdf->Get<std::string>("belt_link");
    beltLink_ = model_.LinkByName(ecm, beltLinkName_);
    if (beltLink_ == gz::kNullEntity) {
      ignerr << "[ConveyorSystem] link [" << beltLinkName_ << "] not found in model " << modelName << "\n";
      return;
    }
    auto size = sdf->Get<ignition::math::Vector2d>("surface_size", ignition::math::Vector2d(2.0, 0.5)).first;
    halfLen_ = size.X() / 2.0;
    halfWid_ = size.Y() / 2.0;
    offset_ = sdf->Get<ignition::math::Vector3d>("surface_offset", ignition::math::Vector3d::Zero).first;
    dir_ = sdf->Get<ignition::math::Vector3d>("direction", ignition::math::Vector3d::UnitX).first.Normalized();
    captureHeight_ = sdf->Get<double>("capture_height", 0.08).first;
    targetVel_ = sdf->Get<double>("initial_velocity", 0.0).first;
    maxAccel_ = sdf->Get<double>("max_acceleration", 2.0).first;
    modelPrefix_ = sdf->Get<std::string>("model_prefix", "").first;
    const std::string cmdTopic = sdf->Get<std::string>("cmd_topic", "/model/" + modelName + "/conveyor/cmd_vel").first;
    const std::string stateTopic =
        sdf->Get<std::string>("state_topic", "/model/" + modelName + "/conveyor/state").first;
    node_.Subscribe(cmdTopic, &ConveyorSystem::OnCmd, this);
    statePub_ = node_.Advertise<ignition::msgs::Double>(stateTopic);
    valid_ = true;
    ignmsg << "[ConveyorSystem] " << modelName << "/" << beltLinkName_ << " footprint " << size << " cmd " << cmdTopic
           << "\n";
  }

  void PreUpdate(const gz::UpdateInfo& info, gz::EntityComponentManager& ecm) override {
    if (!valid_ || info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    {
      std::lock_guard<std::mutex> lk(mutex_);
      const double dv = targetVel_ - currentVel_;
      const double step = maxAccel_ * dt;
      currentVel_ += std::clamp(dv, -step, step);
    }
    // 50 Hz encoder
    if (info.simTime - lastPub_ >= std::chrono::milliseconds(20)) {
      ignition::msgs::Double m;
      m.set_data(currentVel_);
      statePub_.Publish(m);
      lastPub_ = info.simTime;
    }

    const ignition::math::Pose3d beltPose = gz::worldPose(beltLink_, ecm);
    const ignition::math::Vector3d dirWorld = beltPose.Rot().RotateVector(dir_);
    const ignition::math::Vector3d vWorld = dirWorld * currentVel_;

    // models currently welded to something (vacuum gripper) are not on the belt
    std::set<gz::Entity> held;
    ecm.Each<gz::components::DetachableJoint>([&](const gz::Entity&, const gz::components::DetachableJoint* dj) {
      auto parent = ecm.Component<gz::components::ParentEntity>(dj->Data().childLink);
      if (parent) held.insert(parent->Data());
      return true;
    });

    std::set<gz::Entity> onBelt;
    ecm.Each<gz::components::Model, gz::components::Name, gz::components::Pose>(
        [&](const gz::Entity& e, const gz::components::Model*, const gz::components::Name* name,
            const gz::components::Pose*) {
          if (e == model_.Entity()) return true;
          auto st = ecm.Component<gz::components::Static>(e);
          if (st && st->Data()) return true;
          if (!modelPrefix_.empty() && name->Data().rfind(modelPrefix_, 0) != 0) return true;
          if (held.count(e)) return true;
          // nested models are skipped (only top-level models are carried)
          auto parent = ecm.Component<gz::components::ParentEntity>(e);
          if (parent && ecm.Component<gz::components::Model>(parent->Data())) return true;

          const ignition::math::Pose3d mp = gz::worldPose(e, ecm);
          const ignition::math::Vector3d local = beltPose.Rot().RotateVectorReverse(mp.Pos() - beltPose.Pos()) - offset_;
          if (std::fabs(local.X()) > halfLen_ || std::fabs(local.Y()) > halfWid_) return true;
          if (local.Z() < -0.01 || local.Z() > captureHeight_) return true;
          onBelt.insert(e);

          gz::Model m(e);
          gz::Entity canonical = m.CanonicalLink(ecm);
          gz::Link link(canonical);
          link.EnableVelocityChecks(ecm, true);
          double vz = 0.0;
          if (auto wv = link.WorldLinearVelocity(ecm)) vz = wv->Z();
          ignition::math::Vector3d cmdWorld(vWorld.X(), vWorld.Y(), vz);
          // LinearVelocityCmd on a model is expressed in the model frame
          ignition::math::Vector3d cmdLocal = mp.Rot().RotateVectorReverse(cmdWorld);
          auto comp = ecm.Component<gz::components::LinearVelocityCmd>(e);
          if (comp)
            comp->Data() = cmdLocal;
          else
            ecm.CreateComponent(e, gz::components::LinearVelocityCmd(cmdLocal));
          return true;
        });

    // release models that left the belt so normal dynamics resume
    for (auto e : carried_) {
      if (!onBelt.count(e) && ecm.HasEntity(e)) ecm.RemoveComponent<gz::components::LinearVelocityCmd>(e);
    }
    carried_.swap(onBelt);
  }

 private:
  void OnCmd(const ignition::msgs::Double& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    targetVel_ = msg.data();
  }

  gz::Model model_{gz::kNullEntity};
  gz::Entity beltLink_{gz::kNullEntity};
  std::string beltLinkName_, modelPrefix_;
  double halfLen_{1.0}, halfWid_{0.25}, captureHeight_{0.08};
  ignition::math::Vector3d offset_{0, 0, 0}, dir_{1, 0, 0};
  double targetVel_{0.0}, currentVel_{0.0}, maxAccel_{2.0};
  bool valid_{false};
  std::mutex mutex_;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher statePub_;
  std::chrono::steady_clock::duration lastPub_{0};
  std::set<gz::Entity> carried_;
};

}  // namespace togsim

IGNITION_ADD_PLUGIN(togsim::ConveyorSystem, ignition::gazebo::System, togsim::ConveyorSystem::ISystemConfigure,
                    togsim::ConveyorSystem::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(togsim::ConveyorSystem, "togsim::ConveyorSystem")
