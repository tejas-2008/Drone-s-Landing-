#ifndef MOVING_PLATFORM_PLUGIN_HPP
#define MOVING_PLATFORM_PLUGIN_HPP

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <platform_control/SetPlatformVelocity.h>
#include <ros/ros.h>

namespace gazebo {

class MovingPlatformPlugin : public ModelPlugin {
public:
  MovingPlatformPlugin() = default;
  virtual ~MovingPlatformPlugin() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate();

  bool SetVelocityCb(platform_control::SetPlatformVelocity::Request &req,
                     platform_control::SetPlatformVelocity::Response &res);

private:
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::ServiceServer velocity_service_;

  double vx_{0.0};
  double vy_{0.0};
  double vz_{0.0};
  double yaw_rate_{0.0};
};

} // namespace gazebo

#endif
