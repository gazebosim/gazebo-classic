/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODETypes.hh"

using namespace gazebo;

class TireSlipTest : public ServerFixture
{
  /// \brief Set joint commands for tire testrig.
  /// \param[in] _wheelSpeed Wheel spin speed in rad/s.
  /// \param[in] _drumSpeed Drum spin speed in rad/s.
  /// \param[in] _suspForce Suspension force to apply in N.
  /// \param[in] _steer Steer angle to apply.
  public: void SetCommands(const double _wheelSpeed, const double _drumSpeed,
                           const double _suspForce, const math::Angle _steer);

  /// \brief Publisher of joint commands for the tire model.
  protected: transport::PublisherPtr tireJointCmdPub;

  /// \brief Publisher of joint commands for the drum model.
  protected: transport::PublisherPtr drumJointCmdPub;

  /// \brief Joint pointer for steering joint.
  protected: physics::JointPtr steerJoint;
};

/////////////////////////////////////////////////
TEST_F(TireSlipTest, Logitudinal)
{
  const double metersPerMile = 1609.34;
  const double secondsPerHour = 3600.0;

  Load("worlds/tire_drum_test.world", true);

  // joint command publishers
  this->tireJointCmdPub = node->Advertise<msgs::JointCmd>("~/tire/joint_cmd");
  this->drumJointCmdPub = node->Advertise<msgs::JointCmd>("~/drum/joint_cmd");

  sensors::ForceTorqueSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        sensors::get_sensor("default::tire::axel_wheel::force_torque"));

  ASSERT_TRUE(sensor != NULL);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  math::Vector3 g = physics->GetGravity();

  physics::ModelPtr wheelModel = world->GetModel("tire");
  ASSERT_TRUE(wheelModel != NULL);

  physics::LinkPtr wheelLink = wheelModel->GetLink("wheel");
  ASSERT_TRUE(wheelLink != NULL);

  double wheelMass = 0.0;
  double wheelRadius = 0.0;
  double wheelStiffness = 0.0;
  {
    wheelMass = wheelLink->GetInertial()->GetMass();

    physics::CollisionPtr wheelCollision = wheelLink->GetCollision("collision");
    ASSERT_TRUE(wheelCollision != NULL);

    physics::ShapePtr shape = wheelCollision->GetShape();
    ASSERT_TRUE(shape != NULL);
    ASSERT_TRUE(shape->HasType(physics::Base::CYLINDER_SHAPE)
             || shape->HasType(physics::Base::SPHERE_SHAPE));
    if (shape->HasType(physics::Base::CYLINDER_SHAPE))
    {
      physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(shape.get());
      wheelRadius = cyl->GetRadius();
    }
    else if (shape->HasType(physics::Base::SPHERE_SHAPE))
    {
      physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(shape.get());
      wheelRadius = sph->GetRadius();
    }

    physics::ODESurfaceParamsPtr surface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(
        wheelCollision->GetSurface());
    ASSERT_TRUE(surface != NULL);
    wheelStiffness = surface->kp;
  }

  double modelMass = 0.0;
  {
    physics::Link_V links = wheelModel->GetLinks();
    for (physics::Link_V::iterator iter = links.begin();
         iter != links.end(); ++iter)
    {
      modelMass += (*iter)->GetInertial()->GetMass();
    }
  }

  double drumRadius = 0.0;
  {
    physics::ModelPtr drumModel = world->GetModel("drum");
    ASSERT_TRUE(drumModel != NULL);

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

  this->steerJoint =  wheelModel->GetJoint("steer");
  ASSERT_TRUE(this->steerJoint != NULL);

  // Measure certain quantities
  math::SignalMaxAbsoluteValue statsDrumSpeed;
  math::SignalMaxAbsoluteValue statsVerticalForce;
  math::SignalMaxAbsoluteValue statsHeight;
  math::SignalMaxAbsoluteValue statsSteer;
  math::SignalMaxAbsoluteValue statsWheelSpeed;

  // speed in miles / hour, convert to rad/s
  const double wheelSpeed = 25.0 * metersPerMile / secondsPerHour / wheelRadius;
  const double drumSpeed = -25.0 * metersPerMile / secondsPerHour /  drumRadius;
  const double suspForce = 1000.0;
  math::Angle steer;
  steer.SetFromDegree(0.0);

  this->SetCommands(wheelSpeed, drumSpeed, suspForce, steer);
  common::Time::MSleep(100);
  world->Step(150);

  for (int i = 0; i < 1e3; ++i)
  {
    world->Step(1);
    statsHeight.InsertData(wheelLink->GetWorldPose().pos.z
      - (wheelRadius - suspForce / wheelStiffness));
    statsSteer.InsertData((this->steerJoint->GetAngle(0) - steer).Radian());
    statsVerticalForce.InsertData(
      sensor->GetForce().z - (suspForce - (modelMass-wheelMass) * g.z));
  }
  EXPECT_LT(statsHeight.Value(), 1e-3);
  EXPECT_LT(statsSteer.Value(), 1e-2);
  EXPECT_LT(statsVerticalForce.Value(), suspForce * 1e-2);
}

/////////////////////////////////////////////////
void TireSlipTest::SetCommands(const double _wheelSpeed,
  const double _drumSpeed, const double _suspForce, const math::Angle _steer)
{
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
    pid->set_target(_drumSpeed);
    pid->set_p_gain(drumSpinP);
    pid->set_i_gain(drumSpinI);
    pid->set_d_gain(drumSpinD);

    this->drumJointCmdPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::axel_wheel");

    msgs::PID *pid = msg.mutable_velocity();
    pid->set_target(_wheelSpeed);
    pid->set_p_gain(wheelSpinP);
    pid->set_i_gain(wheelSpinI);
    pid->set_d_gain(wheelSpinD);

    this->tireJointCmdPub->Publish(msg);
  }

  {
    msgs::JointCmd msg;
    msg.set_name("tire::world_upright");
    msg.set_force(-_suspForce);

    this->tireJointCmdPub->Publish(msg);
  }

  this->steerJoint->SetHighStop(0, _steer);
  this->steerJoint->SetLowStop(0, _steer);
}

// /////////////////////////////////////////////////
// TEST_F(TireSlipTest, Angle15Degrees)
// {
//   Load("worlds/tire_drum_steer_15_test.world");
// 
//   transport::PublisherPtr tireJointCmdPub = node->Advertise<msgs::JointCmd>(
//       "~/tire/joint_cmd");
// 
//   transport::PublisherPtr drumJointCmdPub = node->Advertise<msgs::JointCmd>(
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
//     tireJointCmdPub->Publish(jointCmdMsg);
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
//     drumJointCmdPub->Publish(jointCmdMsg);
//   }
//   common::Time::Sleep(10);
// }

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
