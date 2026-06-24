#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class WindGeneratorPoseLockPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr /*sdf*/) override
  {
    if (!model)
    {
      gzerr << "[wind_generator_pose_lock] Missing model pointer.\n";
      return;
    }

    model_ = model;
    locked_pose_ = model_->WorldPose();
    base_link_ = model_->GetLink("wind_generator_base_link");
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WindGeneratorPoseLockPlugin::OnUpdate, this));

    gzmsg << "[wind_generator_pose_lock] Locking " << model_->GetName()
          << " at " << locked_pose_ << "\n";
  }

private:
  void OnUpdate()
  {
    if (!model_)
    {
      return;
    }

    model_->SetWorldPose(locked_pose_);
    if (base_link_)
    {
      base_link_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      base_link_->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  ignition::math::Pose3d locked_pose_;
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(WindGeneratorPoseLockPlugin)
}  // namespace gazebo
