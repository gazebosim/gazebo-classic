/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Road2d.hh"

using namespace gazebo;

class RoadTest : public ServerFixture
{
  /// \brief Test loading a world with textured roads
  public: void TexturedWorld();

  /// \brief Test creating a road visual
  public: void RoadVisual();
};

/////////////////////////////////////////////////
void RoadTest::TexturedWorld()
{
  // Load the sample world
  Load("worlds/road_textures.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->Step(steps);
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

/////////////////////////////////////////////////
TEST_F(RoadTest, TexturedWorld)
{
  TexturedWorld();
}

/////////////////////////////////////////////////
void RoadTest::RoadVisual()
{
  Load("worlds/empty.world");

  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  rendering::VisualPtr worldVis = scene->WorldVisual();
  ASSERT_TRUE(worldVis != nullptr);

  // spawn a camera sensor to trigger render events needed to process road msgs
  ignition::math::Pose3d testPose(
    ignition::math::Vector3d(-5, 0, 5),
    ignition::math::Quaterniond::Identity);
  SpawnCamera("cam_model", "cam_sensor", testPose.Pos(),
      testPose.Rot().Euler());

  // publish road msg to create the road
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::PublisherPtr roadPub = node->Advertise<msgs::Road>("~/roads");
  roadPub->WaitForConnection();

  std::string roadName = "road_test";
  msgs::Road msg;
  msg.set_name(roadName);
  msg.set_width(8);
  msgs::Vector3d *pt01Msg = msg.add_point();
  msgs::Set(pt01Msg, ignition::math::Vector3d(0, 0, 0));
  msgs::Vector3d *pt02Msg = msg.add_point();
  msgs::Set(pt02Msg, ignition::math::Vector3d(0, 5, 0));
  msgs::Vector3d *pt03Msg = msg.add_point();
  msgs::Set(pt03Msg, ignition::math::Vector3d(2, 5, 0));

  roadPub->Publish(msg);

  // verify that the road visual is created
  rendering::Road2dPtr roadVis;
  int sleep = 0;
  int maxSleep = 20;
  while (!roadVis && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(30);
    for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
    {
      rendering::VisualPtr childVis = worldVis->GetChild(i);
      roadVis = std::dynamic_pointer_cast<rendering::Road2d>(childVis);
      if (roadVis)
        break;
    }
    sleep++;
  }
  ASSERT_TRUE(roadVis != nullptr);
  EXPECT_EQ(roadVis->GetName(), roadName);
}

/////////////////////////////////////////////////
TEST_F(RoadTest, RoadVisual)
{
  RoadVisual();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
