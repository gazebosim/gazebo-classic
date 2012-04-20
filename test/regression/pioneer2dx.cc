/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "physics/physics.h"

using namespace gazebo;
class Pioneer2dx : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(PR2Test, StraightLine)
{
  Load("world/pioneer2dx.world");
  transport::PublisherPtr velPub = this->node->Publish<gazebo::msgs::Pose>(
      "~/pioneer2dx/vel_cmd");

  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(msg.mutable_position(),
      gazebo::math::Vector3(cmd->vel.px, cmd->vel.py, 0));
  gazebo::msgs::Set(msg.mutable_orientation(),
      gazebo::math::Quaternion(0, 0, cmd->vel.pa));
  this->velPub->Publish(msg);

  math::Pose startPose, endPose;
  startPose = this->poses["pioneer2dx"];

  common::Time startTime = this->simTime;
  common::Time currTime = this->simTime;
  while (currTime - startTime < common::Time(10,0))
  {
    usleep(1000);
    currTime = this->simTime;
  }

  std::cout << "Start[" << startTime << "] Curr[" << currTime << "]\n";
  std::cout << "StartPose[" << startPose << "] EndPose[" << endPose << "]\n";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
