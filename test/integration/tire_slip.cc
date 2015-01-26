/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

class TireSlipTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(TireSlipTest, Logitudinal)
{
  const double metersPerMile = 1609.34;
  const double secondsPerHour = 3600.0;

  Load("worlds/tire_drum_steer_15_test.world", true);

  // PID Controller for the tire
  transport::PublisherPtr tirePIDPub = node->Advertise<msgs::JointCmd>(
      "~/tire/joint_cmd");

  // PID Controller for the drum
  transport::PublisherPtr drumPIDPub = node->Advertise<msgs::JointCmd>(
      "~/drum/joint_cmd");

  sensors::ForceTorqueSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        sensors::get_sensor("default::tire::axel_wheel::force_torque"));

  ASSERT_TRUE(sensor != NULL);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr wheelModel = world->GetModel("tire");
  ASSERT_TRUE(wheelModel != NULL);

  double wheelRadius = 0.0;
  {
    physics::LinkPtr wheelLink = wheelModel->GetLink("wheel");
    ASSERT_TRUE(wheelLink != NULL);

    physics::CollisionPtr wheelCollision = wheelLink->GetCollision("collision");
    ASSERT_TRUE(wheelCollision != NULL);

    physics::ShapePtr shape = wheelCollision->GetShape();
    ASSERT_TRUE(shape != NULL);
    ASSERT_TRUE(shape->HasType(physics::Base::CYLINDER_SHAPE));
    physics::CylinderShape *cyl =
      static_cast<physics::CylinderShape*>(shape.get());
    wheelRadius = cyl->GetRadius();
  }

  physics::ModelPtr drumModel = world->GetModel("drum");
  ASSERT_TRUE(drumModel != NULL);

  double drumRadius = 0.0;
  {
    physics::LinkPtr drumLink = drumModel->GetLink("link");
    ASSERT_TRUE(drumLink != NULL);

    physics::CollisionPtr drumCollision = drumLink->GetCollision("collision");
    ASSERT_TRUE(drumCollision != NULL);

    physics::ShapePtr shape = drumCollision->GetShape();
    ASSERT_TRUE(shape != NULL);
    ASSERT_TRUE(shape->HasType(physics::Base::CYLINDER_SHAPE));
    physics::CylinderShape *cyl =
      static_cast<physics::CylinderShape*>(shape.get());
    drumRadius = cyl->GetRadius();
  }

  physics::JointPtr steerJoint =  wheelModel->GetJoint("steer");
  ASSERT_TRUE(steerJoint != NULL);

  physics::JointPtr worldUprightJoint =  wheelModel->GetJoint("world_upright");
  ASSERT_TRUE(worldUprightJoint != NULL);

  // speed in miles / hour, convert to rad/s
  const double wheelSpeed = 25.0 * metersPerMile / secondsPerHour / wheelRadius;
  const double drumSpeed = -25.0 * metersPerMile / secondsPerHour /  drumRadius;
  const double normalForce = 200.0;
  math::Angle steer;
  steer.SetFromDegree(-15.0);

  // PID gains for joint controllers
  const double wheelSpinP = 1e1;
  const double wheelSpinI = 0.0;
  const double wheelSpinD = 0.0;
  const double drumSpinP = 1e2;
  const double drumSpinI = 0.0;
  const double drumSpinD = 0.0;

  {
    msgs::JointCmd msg;
    msg.set_name("drum::joint");

    msgs::PID *pid = msg.mutable_velocity();
    pid->set_target(drumSpeed);
    pid->set_p_gain(drumSpinP);
    pid->set_i_gain(drumSpinI);
    pid->set_d_gain(drumSpinD);

    drumPIDPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::axel_wheel");

    msgs::PID *pid = msg.mutable_velocity();
    pid->set_target(wheelSpeed);
    pid->set_p_gain(wheelSpinP);
    pid->set_i_gain(wheelSpinI);
    pid->set_d_gain(wheelSpinD);

    tirePIDPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::world_upright");
    msg.set_force(-normalForce);

    tirePIDPub->Publish(msg);
  }

  steerJoint->SetHighStop(0, steer);
  steerJoint->SetLowStop(0, steer);

  common::Time::MSleep(100);

  for (int i = 0; i < 10000; ++i)
  {
    world->Step(10);
    std::cout << "I[" << i << "] Torque[" << sensor->GetTorque()
              << "] Force[" << sensor->GetForce() << "]\n";
  }
}

// /////////////////////////////////////////////////
// TEST_F(TireSlipTest, Angle15Degrees)
// {
//   Load("worlds/tire_drum_steer_15_test.world");
// 
//   transport::PublisherPtr tirePIDPub = node->Advertise<msgs::JointCmd>(
//       "~/tire/joint_cmd");
// 
//   transport::PublisherPtr drumPIDPub = node->Advertise<msgs::JointCmd>(
//       "~/drum/joint_cmd");
// 
//   common::Time::Sleep(10);
//   {
//     gazebo::msgs::JointCmd jointCmdMsg;
//     gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_position();
// 
//     jointCmdMsg.set_name("tire::world_upright");
//     pidMsg->set_target(GZ_DTOR(15));
//     pidMsg->set_p_gain(10.0);
//     pidMsg->set_d_gain(0.0);
//     pidMsg->set_i_gain(0.0);
// 
//     tirePIDPub->Publish(jointCmdMsg);
//   }
// 
//   {
//     gazebo::msgs::JointCmd jointCmdMsg;
//     gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_velocity();
// 
//     jointCmdMsg.set_name("drum::joint");
//     pidMsg->set_target(1.0);
//     pidMsg->set_p_gain(10.0);
//     pidMsg->set_d_gain(0.0);
//     pidMsg->set_i_gain(0.0);
// 
//     drumPIDPub->Publish(jointCmdMsg);
//   }
//   common::Time::Sleep(10);
// }

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
