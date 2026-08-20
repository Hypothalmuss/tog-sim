// tog-sim VacuumGripperSystem — a vacuum cup that can grab ANY product at runtime.
//
// Attach to the robot model. A contact sensor on the cup collision reports what the cup touches. When the vacuum is
// commanded ON and the cup touches a model whose name starts with <model_prefix>, a fixed DetachableJoint is created
// between the cup link and the product's link after <seal_delay> seconds (the time a real ejector needs to build
// vacuum). OFF removes the joint after <release_delay>. Products heavier than <max_payload> cannot be sealed.
//
// Command (ignition.msgs.StringMsg on <cmd_topic>):  "on" | "on:<model_name>" | "off"
// State  (ignition.msgs.StringMsg on <state_topic>): "commanded=<0|1> sealed=<0|1> model=<name|->"  @ 100 Hz
//
// Why custom: the stock DetachableJoint system needs the child model name in SDF at load time, so it cannot grab
// products spawned later. This system performs the same ECM operation (components::DetachableJoint) dynamically.
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <mutex>
#include <string>

namespace togsim {

namespace gz = ignition::gazebo;

class VacuumGripperSystem : public gz::System,
                            public gz::ISystemConfigure,
                            public gz::ISystemPreUpdate {
 public:
  void Configure(const gz::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::EntityComponentManager& ecm, gz::EventManager&) override {
    model_ = gz::Model(entity);
    cupLinkName_ = sdf->Get<std::string>("cup_link", "vgc10_suction").first;
    cupCollisionName_ = sdf->Get<std::string>("cup_collision", cupLinkName_ + "_collision").first;
    modelPrefix_ = sdf->Get<std::string>("model_prefix", "product_").first;
    sealDelay_ = sdf->Get<double>("seal_delay", 0.03).first;
    releaseDelay_ = sdf->Get<double>("release_delay", 0.02).first;
    maxPayload_ = sdf->Get<double>("max_payload", 1.0).first;
    const std::string cmdTopic = sdf->Get<std::string>("cmd_topic", "/togsim/vacuum/cmd").first;
    const std::string stateTopic = sdf->Get<std::string>("state_topic", "/togsim/vacuum/state").first;
    node_.Subscribe(cmdTopic, &VacuumGripperSystem::OnCmd, this);
    statePub_ = node_.Advertise<ignition::msgs::StringMsg>(stateTopic);
    ignmsg << "[VacuumGripperSystem] cup link [" << cupLinkName_ << "] collision [" << cupCollisionName_
           << "] cmd " << cmdTopic << "\n";
  }

  void PreUpdate(const gz::UpdateInfo& info, gz::EntityComponentManager& ecm) override {
    if (info.paused) return;
    if (cupLink_ == gz::kNullEntity) {
      cupLink_ = model_.LinkByName(ecm, cupLinkName_);
      if (cupLink_ == gz::kNullEntity) {
        // URDF fixed-joint lumping may have renamed things; report once
        if (!warned_) {
          ignwarn << "[VacuumGripperSystem] link [" << cupLinkName_ << "] not found yet\n";
          warned_ = true;
        }
        return;
      }
      // find the cup collision entity by name among the link's children
      ecm.Each<gz::components::Collision, gz::components::Name, gz::components::ParentEntity>(
          [&](const gz::Entity& e, const gz::components::Collision*, const gz::components::Name* n,
              const gz::components::ParentEntity* p) {
            if (p->Data() == cupLink_ && n->Data() == cupCollisionName_) cupCollision_ = e;
            return true;
          });
      if (cupCollision_ == gz::kNullEntity) {
        ignerr << "[VacuumGripperSystem] collision [" << cupCollisionName_ << "] not found on link ["
               << cupLinkName_ << "]. Contact-based sealing disabled.\n";
      } else {
        ignmsg << "[VacuumGripperSystem] ready (cup collision entity " << cupCollision_ << ")\n";
      }
    }

    bool cmdOn;
    std::string cmdTarget;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cmdOn = cmdOn_;
      cmdTarget = cmdTarget_;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();

    if (cmdOn && joint_ == gz::kNullEntity) {
      gz::Entity touching = TouchedProductLink(ecm, cmdTarget);
      if (touching == gz::kNullEntity) {
        sealStart_ = -1.0;
      } else {
        if (sealStart_ < 0) sealStart_ = t;
        if (t - sealStart_ >= sealDelay_) {
          double mass = 0.0;
          if (auto in = ecm.Component<gz::components::Inertial>(touching)) mass = in->Data().MassMatrix().Mass();
          if (mass > maxPayload_) {
            if (!payloadWarned_) {
              ignwarn << "[VacuumGripperSystem] product too heavy (" << mass << " kg > " << maxPayload_ << ")\n";
              payloadWarned_ = true;
            }
          } else {
            joint_ = ecm.CreateEntity();
            ecm.CreateComponent(joint_, gz::components::DetachableJoint({cupLink_, touching, "fixed"}));
            auto modelEnt = ecm.Component<gz::components::ParentEntity>(touching)->Data();
            attachedModel_ = ecm.Component<gz::components::Name>(modelEnt)->Data();
            attachedLink_ = touching;
            ignmsg << "[VacuumGripperSystem] sealed on " << attachedModel_ << "\n";
          }
        }
      }
    } else if (!cmdOn) {
      sealStart_ = -1.0;
      payloadWarned_ = false;
      if (joint_ != gz::kNullEntity) {
        if (releaseStart_ < 0) releaseStart_ = t;
        if (t - releaseStart_ >= releaseDelay_) {
          ecm.RequestRemoveEntity(joint_, true);
          ignmsg << "[VacuumGripperSystem] released " << attachedModel_ << "\n";
          joint_ = gz::kNullEntity;
          attachedModel_.clear();
          attachedLink_ = gz::kNullEntity;
          releaseStart_ = -1.0;
        }
      }
    }
    // product vanished (despawned) while held
    if (joint_ != gz::kNullEntity && !ecm.HasEntity(attachedLink_)) {
      ecm.RequestRemoveEntity(joint_, true);
      joint_ = gz::kNullEntity;
      attachedModel_.clear();
    }

    if (info.simTime - lastPub_ >= std::chrono::milliseconds(10)) {
      ignition::msgs::StringMsg m;
      m.set_data("commanded=" + std::string(cmdOn ? "1" : "0") + " sealed=" +
                 std::string(joint_ != gz::kNullEntity ? "1" : "0") +
                 " model=" + (attachedModel_.empty() ? "-" : attachedModel_));
      statePub_.Publish(m);
      lastPub_ = info.simTime;
    }
  }

 private:
  // Returns the link entity of a product collision currently in contact with the cup (kNullEntity if none).
  gz::Entity TouchedProductLink(gz::EntityComponentManager& ecm, const std::string& target) {
    if (cupCollision_ == gz::kNullEntity) return gz::kNullEntity;
    auto data = ecm.Component<gz::components::ContactSensorData>(cupCollision_);
    if (!data) return gz::kNullEntity;
    for (const auto& contact : data->Data().contact()) {
      gz::Entity other = (contact.collision1().id() == cupCollision_) ? contact.collision2().id()
                                                                      : contact.collision1().id();
      auto linkComp = ecm.Component<gz::components::ParentEntity>(other);
      if (!linkComp) continue;
      gz::Entity link = linkComp->Data();
      auto modelComp = ecm.Component<gz::components::ParentEntity>(link);
      if (!modelComp) continue;
      auto nameComp = ecm.Component<gz::components::Name>(modelComp->Data());
      if (!nameComp) continue;
      const std::string& name = nameComp->Data();
      if (name.rfind(modelPrefix_, 0) != 0) continue;
      if (!target.empty() && target != "auto" && name != target) continue;
      return link;
    }
    return gz::kNullEntity;
  }

  void OnCmd(const ignition::msgs::StringMsg& msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    const std::string& s = msg.data();
    if (s.rfind("on", 0) == 0) {
      cmdOn_ = true;
      cmdTarget_ = (s.size() > 3 && s[2] == ':') ? s.substr(3) : "";
    } else {
      cmdOn_ = false;
      cmdTarget_.clear();
    }
  }

  gz::Model model_{gz::kNullEntity};
  gz::Entity cupLink_{gz::kNullEntity}, cupCollision_{gz::kNullEntity};
  gz::Entity joint_{gz::kNullEntity}, attachedLink_{gz::kNullEntity};
  std::string cupLinkName_, cupCollisionName_, modelPrefix_, attachedModel_;
  double sealDelay_{0.03}, releaseDelay_{0.02}, maxPayload_{1.0};
  double sealStart_{-1.0}, releaseStart_{-1.0};
  bool warned_{false}, payloadWarned_{false};
  bool cmdOn_{false};
  std::string cmdTarget_;
  std::mutex mutex_;
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher statePub_;
  std::chrono::steady_clock::duration lastPub_{0};
};

}  // namespace togsim

IGNITION_ADD_PLUGIN(togsim::VacuumGripperSystem, ignition::gazebo::System,
                    togsim::VacuumGripperSystem::ISystemConfigure, togsim::VacuumGripperSystem::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(togsim::VacuumGripperSystem, "togsim::VacuumGripperSystem")
