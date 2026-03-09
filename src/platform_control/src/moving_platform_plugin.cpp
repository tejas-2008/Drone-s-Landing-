#include "platform_control/moving_platform_plugin.hpp"

namespace gazebo
{

void MovingPlatformPlugin::Load(physics::ModelPtr model, sdf::ElementPtr)
{
  model_ = model;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "moving_platform_plugin",
              ros::init_options::NoSigintHandler);
  }

  nh_.reset(new ros::NodeHandle("platform"));

  velocity_service_ = nh_->advertiseService(
    "set_velocity",
    &MovingPlatformPlugin::SetVelocityCb,
    this
  );

  static ros::AsyncSpinner spinner(1);
  spinner.start();

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&MovingPlatformPlugin::OnUpdate, this)
  );

  ROS_INFO("MovingPlatformPlugin Loaded");


}

bool MovingPlatformPlugin::SetVelocityCb(
  platform_control::SetPlatformVelocity::Request &req,
  platform_control::SetPlatformVelocity::Response &res)
{
  vx_ = req.vx;
  vy_ = req.vy;
  vz_ = req.vz;
  yaw_rate_ = req.yaw_rate;

  res.success = true;
  res.message = "Platform velocity updated";

  return true;
}

void MovingPlatformPlugin::OnUpdate()
{
  ignition::math::Pose3d pose = model_->WorldPose();
  double dt = model_->GetWorld()->Physics()->GetMaxStepSize();

  pose.Pos().X() += vx_ * dt;
  pose.Pos().Y() += vy_ * dt;
  pose.Pos().Z() += vz_ * dt;

  auto rpy = pose.Rot().Euler();
  rpy.Z() += yaw_rate_ * dt;
  pose.Rot() = ignition::math::Quaterniond(rpy);

  model_->SetWorldPose(pose);
}

GZ_REGISTER_MODEL_PLUGIN(MovingPlatformPlugin)

}  // namespace gazebo
