// Persistent Ignition Transport client exposed as one generic ROS 2 service (togsim_msgs/srv/IgnService).
//
// Why: the one-shot `ign service` CLI creates a new transport node per call (discovery on every call) and, when the
// Fortress server is busy, its reply arrives after the CLI gave up ("Host unreachable" in the server log). The request
// was usually executed, so callers could not tell success from failure: removed products leaked on the belts,
// the product spawner starved the cell, benchmarks decayed. One long-lived node fixes the discovery cost and makes the
// reply reliable; callers keep the protobuf text-format request strings they already use.
#include <google/protobuf/text_format.h>

#include <ignition/msgs/Factory.hh>
#include <ignition/transport/Node.hh>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "togsim_msgs/srv/ign_service.hpp"

class IgnServiceBridge : public rclcpp::Node {
 public:
  IgnServiceBridge() : Node("ign_service_bridge") {
    srv_ = create_service<togsim_msgs::srv::IgnService>(
        "/togsim/ign_service",
        [this](const std::shared_ptr<togsim_msgs::srv::IgnService::Request> req,
               std::shared_ptr<togsim_msgs::srv::IgnService::Response> res) { handle(*req, *res); });
    RCLCPP_INFO(get_logger(), "ign_service_bridge ready (/togsim/ign_service)");
  }

 private:
  void handle(const togsim_msgs::srv::IgnService::Request& req, togsim_msgs::srv::IgnService::Response& res) {
    res.success = false;
    auto msg = ignition::msgs::Factory::New(req.request_type);
    if (!msg) {
      res.response = "unknown request type " + req.request_type;
      return;
    }
    if (!req.request.empty() && !google::protobuf::TextFormat::ParseFromString(req.request, msg.get())) {
      res.response = "cannot parse request text for " + req.request_type;
      return;
    }
    std::string data;
    msg->SerializeToString(&data);
    std::string reply;
    bool result = false;
    const unsigned int timeout_ms = req.timeout_s > 0.0f ? static_cast<unsigned int>(req.timeout_s * 1000.0f) : 5000u;
    if (!node_.RequestRaw(req.service, data, req.request_type, req.response_type, timeout_ms, reply, result)) {
      res.response = "timeout";
      return;
    }
    auto rep = ignition::msgs::Factory::New(req.response_type);
    if (rep && rep->ParseFromString(reply)) {
      google::protobuf::TextFormat::PrintToString(*rep, &res.response);
    } else {
      res.response = "";
    }
    // transport result, and for Boolean replies also the payload (the server answers false on a failed create/remove)
    res.success = result;
    if (req.response_type == "ignition.msgs.Boolean") res.success = result && res.response.find("data: true") != std::string::npos;
  }

  ignition::transport::Node node_;
  rclcpp::Service<togsim_msgs::srv::IgnService>::SharedPtr srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IgnServiceBridge>());
  rclcpp::shutdown();
  return 0;
}
