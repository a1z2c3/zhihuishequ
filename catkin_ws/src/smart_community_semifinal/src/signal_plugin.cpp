#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Color.hh>
#include <cmath>
#include <functional>

namespace gazebo {
// Only changes rendered signal lamps. No ROS truth topic is exposed to control.
// Durations are simulation parameters; the new semifinal TXT does not fix them.
class SemifinalSignal : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    offset_ = sdf->HasElement("offset") ? sdf->Get<double>("offset") : 0.0;
    node_.reset(new transport::Node());
    node_->Init(model_->GetWorld()->Name());
    publisher_ = node_->Advertise<msgs::Visual>("~/visual");
    update_ = event::Events::ConnectWorldUpdateBegin(
        [this](const common::UpdateInfo& info) { this->Tick(info.simTime.Double()); });
    gzmsg << "Semifinal signal loaded: " << model_->GetScopedName() << "\n";
  }
 private:
  void Tick(const double now) {
    if (now >= last_ && now-last_ < 0.1) return;
    last_ = now;
    const double phase = std::fmod(now + offset_, 28.0);
    const int active = phase < 10.0 ? 0 : (phase < 25.0 ? 2 : 1);
    const char* names[] = {"red", "yellow", "green"};
    const ignition::math::Color colours[] = {{1,0.01f,0.01f,1}, {1,0.65f,0.01f,1}, {0.01f,1,0.02f,1}};
    for (int i=0; i<3; ++i) {
      const std::string parent = model_->GetScopedName() + "::housing";
      // Preserve Gazebo's real visual id/type rather than creating an incomplete
      // replacement message. Sensor-render scenes resolve material updates by id.
      msgs::Visual message = model_->GetLink("housing")->GetVisualMessage(parent + "::" + names[i]);
      message.set_name(parent + "::" + names[i]);
      message.set_parent_name(parent);
      auto* material = message.mutable_material();
      const ignition::math::Color dark(0.025f,0.025f,0.025f,1.0f);
      msgs::Set(material->mutable_diffuse(), i==active ? colours[i] : dark);
      msgs::Set(material->mutable_ambient(), i==active ? colours[i] : dark);
      msgs::Set(material->mutable_emissive(), i==active ? colours[i] : dark);
      publisher_->Publish(message);
    }
  }
  physics::ModelPtr model_;
  transport::NodePtr node_;
  transport::PublisherPtr publisher_;
  event::ConnectionPtr update_;
  double offset_=0.0, last_=-1.0;
};
GZ_REGISTER_MODEL_PLUGIN(SemifinalSignal)
}
