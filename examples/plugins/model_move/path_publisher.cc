#include <iostream>
#include <math.h>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

int main(int argc, char * argv[])
{
  msgs::PoseAnimation msg;

  msg.set_model_name("box");
  msgs::Pose *p = msg.add_pose();
  msgs::Set(p, math::Pose(5, 5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, math::Pose(5, -5, 0, 0, 0, 0));
  p = msg.add_pose();
  msgs::Set(p, math::Pose(0, 0, 0, 0, 0, 0));

  transport::init();
  transport::run();
  transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");

  gazebo::transport::PublisherPtr pathPub =
    node->Advertise<msgs::PoseAnimation>("/gazebo/default/pose_animation");
  std::cout << "Waiting for connection...\n";
  pathPub->WaitForConnection();
  pathPub->Publish(msg);

  std::cout << "Path published!\n\n";

  gazebo::transport::fini();
  return 0;
}
