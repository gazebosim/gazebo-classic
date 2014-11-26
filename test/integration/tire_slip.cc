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
/*TEST_F(TireSlipTest, AngleZero)
{
  Load("worlds/tire_drum_test.world");

  common::Time::Sleep(5);
  transport::PublisherPtr tirePIDPub = node->Advertise<msgs::JointCmd>(
      "~/tire/joint_cmd");

  transport::PublisherPtr drumPIDPub = node->Advertise<msgs::JointCmd>(
      "~/drum/joint_cmd");

  {
    gazebo::msgs::JointCmd jointCmdMsg;
    gazebo::msgs::PID *pidMsg = jointCmdMsg.mutable_velocity();

    jointCmdMsg.set_name("tire::world_upright");
    pidMsg->set_target(0.0);
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
}*/

/////////////////////////////////////////////////
TEST_F(TireSlipTest, Angle15Degrees)
{
  Load("worlds/tire_drum_test.world");

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
