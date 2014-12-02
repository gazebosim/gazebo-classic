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

class TireSlipTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(TireSlipTest, Logitudinal)
{
  double metersPerMile = 1609.34;
  double secondsPerHour = 3600.0;
  double mphTomps = metersPerMile / secondsPerHour;

  double wheelSpeed = (25.0 * mphTomps);
  double wheelAngle = 0;
  double drumSpeed = (40.0 * mphTomps);

  Load("worlds/tire_drum_no_steer_test.world", true);

  // PID Controller for the tire
  transport::PublisherPtr tirePIDPub = node->Advertise<msgs::JointCmd>(
      "~/tire/joint_cmd");

  // PID Controller for the drum
  transport::PublisherPtr drumPIDPub = node->Advertise<msgs::JointCmd>(
      "~/drum/joint_cmd");

  sensors::ForceTorqueSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
        sensors::get_sensor("default::tire::axel_wheel::force_torque"));

  ASSERT_TRUE(sensor);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world);

  /*physics::ModelPtr tireModel = world->GetModel("tire");
  ASSERT_TRUE(tireModel);

  physics::JointPtr worldUprightJoint =  tireModel->GetJoint("world_upright");
  ASSERT_TRUE(worldUprightJoint);

  worldUprightJoint->SetUpperLimit(0, 0.0);
  worldUprightJoint->SetLowerLimit(0, 0.0);
  */

  // Set the tire to face forward (no yaw)
  /*{
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_position();

    jointCmdMsg.set_name("tire::world_upright");
    pidMsg->set_target(wheelAngle);
    pidMsg->set_p_gain(100.0);
    pidMsg->set_d_gain(0.0);
    pidMsg->set_i_gain(0.0);

    tirePIDPub->Publish(jointCmdMsg);
  }*/

  // common::Time::Sleep(5);

  // Set the tire to rotate at a fixed speed (25mph == 11.176m/s).
  {
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_velocity();

    jointCmdMsg.set_name("tire::axel_wheel");
    pidMsg->set_target(-wheelSpeed);
    pidMsg->set_p_gain(10.0);
    pidMsg->set_d_gain(0.0);
    pidMsg->set_i_gain(0.0);

    tirePIDPub->Publish(jointCmdMsg);
  }

  // Set the drum to rotate at a fixed speed.
  {
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_velocity();

    jointCmdMsg.set_name("drum::joint");
    pidMsg->set_target(drumSpeed);
    pidMsg->set_p_gain(10.0);
    pidMsg->set_d_gain(0.0);
    pidMsg->set_i_gain(0.0);

    drumPIDPub->Publish(jointCmdMsg);
  }

  for (int i = 0; i < 100; ++i)
  {
    world->Step(10);
    /*std::cout << "I[" << i << "] Torque[" << sensor->GetTorque()
              << "] Force[" << sensor->GetForce() << "]\n";
              */
  }
}

/////////////////////////////////////////////////
TEST_F(TireSlipTest, Angle15Degrees)
{
  Load("worlds/tire_drum_steer_15_test.world");

  transport::PublisherPtr tirePIDPub = node->Advertise<msgs::JointCmd>(
      "~/tire/joint_cmd");

  transport::PublisherPtr drumPIDPub = node->Advertise<msgs::JointCmd>(
      "~/drum/joint_cmd");

  common::Time::Sleep(10);
  {
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_position();

    jointCmdMsg.set_name("tire::world_upright");
    pidMsg->set_target(GZ_DTOR(15));
    pidMsg->set_p_gain(10.0);
    pidMsg->set_d_gain(0.0);
    pidMsg->set_i_gain(0.0);

    tirePIDPub->Publish(jointCmdMsg);
  }

  {
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_velocity();

    jointCmdMsg.set_name("drum::joint");
    pidMsg->set_target(1.0);
    pidMsg->set_p_gain(10.0);
    pidMsg->set_d_gain(0.0);
    pidMsg->set_i_gain(0.0);

    drumPIDPub->Publish(jointCmdMsg);
  }
  common::Time::Sleep(10);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
