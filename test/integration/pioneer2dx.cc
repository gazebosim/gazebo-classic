/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/common/Time.hh"
#include "gazebo/transport/TransportIface.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class Pioneer2dx : public ServerFixture,
                   public testing::WithParamInterface<const char*>
{
  public: void StraightLine(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void Pioneer2dx::StraightLine(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not handle pioneer2dx model yet, "
          << "Please see issue #866.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not handle pioneer2dx model yet.\n"
          << "Please see issue #912. "
          << "(https://bitbucket.org/osrf/gazebo/issue/912)\n";
    return;
  }

  Load("worlds/pioneer2dx.world", false, _physicsEngine);
  transport::PublisherPtr velPub = this->node->Advertise<gazebo::msgs::Pose>(
      "~/pioneer2dx/vel_cmd");

  int i = 0;
  for (i = 0; i < 1000 && !this->HasEntity("pioneer2dx"); ++i)
    common::Time::MSleep(500);
  ASSERT_LT(i, 1000);

  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(msg.mutable_position(),
      gazebo::math::Vector3(0.2, 0, 0));
  gazebo::msgs::Set(msg.mutable_orientation(),
      gazebo::math::Quaternion(0, 0, 0));
  velPub->Publish(msg);

  math::Pose startPose, endPose;
  startPose = this->poses["pioneer2dx"];

  common::Time startTime = this->simTime;
  common::Time currTime = this->simTime;

  /*struct timespec interval;
  struct timespec remainder;
  interval.tv_sec = 1 / 1000;
  interval.tv_nsec = (1 % 1000) * 1000000;
  */
  while (currTime - startTime < common::Time(20, 0))
  {
    // nanosleep(&interval, &remainder);
    common::Time::MSleep(100);
    currTime = this->simTime;
  }

  endPose = this->poses["pioneer2dx"];

  double dist = (currTime - startTime).Double() * 0.2;
  std::cout << "Dist[" << dist << "]\n";
  std::cout << "EndPose.x[" << endPose.pos.x << "]\n";
  EXPECT_LT(fabs(endPose.pos.x - dist), 0.1);
  EXPECT_LT(fabs(endPose.pos.y), 0.5);
  EXPECT_LT(fabs(endPose.pos.z), 0.01);
}


TEST_P(Pioneer2dx, StraightLine)
{
  StraightLine(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Pioneer2dx, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
