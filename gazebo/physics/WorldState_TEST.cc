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

#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldState.hh"

using namespace gazebo;

class WorldStateTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(WorldStateTest, SDFConstructor)
{
  // Create the state sdf
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='world_state_name'>"
    << "<model name='ground_plane'>"
    << "  <pose>1 1 1 0 0 0</pose>"
    << "  <link name='link'>"
    << "    <pose>0.1 0.2 0.3 0.4 0.5 0.6</pose>"
    << "  </link>"
    << "</model>"
    << "<model name='model_1'>"
    << "  <pose>2 1 1 0 0 0</pose>"
    << "  <link name='link_1'>"
    << "  </link>"
    << "</model>"
    << "<light name='sun'>"
    << "  <pose>10 20 30 0 0 0</pose>"
    << "</light>"
    << "<light name='light_1'>"
    << "  <pose>1 2 3 0 0 0</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  auto worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  // Create the world state
  physics::WorldState worldState(worldElem->GetElement("state"));

  // Check world state against values from the sdf string
  EXPECT_EQ(worldState.GetModelStateCount(), 2u);
  EXPECT_EQ(worldState.LightStateCount(), 2u);

  auto modelStates = worldState.GetModelStates();
  EXPECT_EQ(modelStates.size(), 2u);
  EXPECT_EQ(modelStates["ground_plane"].GetPose(),
      ignition::math::Pose3d(1, 1, 1, 0, 0, 0));
  EXPECT_EQ(modelStates["model_1"].GetPose(),
      ignition::math::Pose3d(2, 1, 1, 0, 0, 0));
  EXPECT_EQ(modelStates["fake_model"].GetPose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  auto lightStates = worldState.LightStates();
  EXPECT_EQ(lightStates.size(), 2u);
  EXPECT_EQ(lightStates["sun"].Pose(),
      ignition::math::Pose3d(10, 20, 30, 0, 0, 0));
  EXPECT_EQ(lightStates["light_1"].Pose(),
      ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
  EXPECT_EQ(lightStates["fake_light"].Pose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
}

//////////////////////////////////////////////////
TEST_F(WorldStateTest, WorldConstructor)
{
  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create the world state
  physics::WorldState worldState(world);

  // Check world state
  EXPECT_EQ(worldState.GetModelStateCount(), 1u);
  EXPECT_EQ(worldState.LightStateCount(), 1u);

  auto modelStates = worldState.GetModelStates();
  EXPECT_EQ(modelStates.size(), 1u);
  EXPECT_EQ(modelStates["ground_plane"].GetPose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  auto lightStates = worldState.LightStates();
  EXPECT_EQ(lightStates.size(), 1u);
  EXPECT_EQ(lightStates["sun"].Pose(),
      ignition::math::Pose3d(0, 0, 10, 0, 0, 0));
}

//////////////////////////////////////////////////
TEST_F(WorldStateTest, FillSDF)
{
  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create the world state
  physics::WorldState worldState(world);

  // Fill SDF
  sdf::ElementPtr filledSDF(new sdf::Element);
  sdf::initFile("state.sdf", filledSDF);
  worldState.FillSDF(filledSDF);

  EXPECT_TRUE(filledSDF->HasAttribute("world_name"));
  EXPECT_EQ(filledSDF->Get<std::string>("world_name"),
      "default");

  EXPECT_TRUE(filledSDF->HasElement("model"));
  EXPECT_EQ(filledSDF->GetElement("model")->Get<std::string>("name"),
      "ground_plane");

  EXPECT_TRUE(filledSDF->HasElement("light"));
  EXPECT_EQ(filledSDF->GetElement("light")->Get<std::string>("name"),
      "sun");
}

//////////////////////////////////////////////////
TEST_F(WorldStateTest, OperatorsNoInsertionsDeletions)
{
  // Load a world
  this->Load("worlds/shapes.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create a world state
  physics::WorldState worldState0(world);

  // Create the "next state"
  // Only checking a case without insertions or deletions
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='ground_plane'>"
    << "  <pose>0 0 0 0 0 0</pose>"
    << "</model>"
    << "<model name='box'>"
    << "  <pose>0 0 0.5 0 0 0</pose>"
    << "</model>"
    << "<model name='sphere'>"
    << "  <pose>2 4 6 0 0 0</pose>"
    << "</model>"
    << "<model name='cylinder'>"
    << "  <pose>3 6 9 0 1.5707 0</pose>"
    << "</model>"
    << "<light name='sun'>"
    << "  <pose>10 20 30 0 0 0</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  auto worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  physics::WorldState worldState1(worldElem->GetElement("state"));

  // Check that the 2 states are of the same world, with same entities
  EXPECT_EQ(worldState0.GetName(), worldState1.GetName());
  EXPECT_EQ(worldState0.GetModelStateCount(), worldState1.GetModelStateCount());
  EXPECT_EQ(worldState0.LightStateCount(), worldState1.LightStateCount());

  // Check subtraction
  auto worldState2 = worldState1 - worldState0;
  EXPECT_FALSE(worldState2.IsZero());

  // Check there were no deletions or insertions
  auto deletions = worldState2.Deletions();
  EXPECT_EQ(deletions.size(), 0u);

  auto insertions = worldState2.Insertions();
  EXPECT_EQ(insertions.size(), 0u);

  // Check that entities which are the same in both states were removed
  EXPECT_EQ(worldState2.GetModelStateCount(), 2u);
  EXPECT_EQ(worldState2.LightStateCount(), 1u);

  auto modelStates = worldState2.GetModelStates();
  EXPECT_EQ(modelStates.size(), 2u);
  EXPECT_EQ(modelStates["sphere"].GetPose(),
      ignition::math::Pose3d(2, 2.5, 5.5, 0, 0, 0));
  EXPECT_EQ(modelStates["cylinder"].GetPose(),
      ignition::math::Pose3d(3, 7.5, 8.5, 0, 0, 0));
  EXPECT_EQ(modelStates["fake_model"].GetPose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  auto lightStates = worldState2.LightStates();
  EXPECT_EQ(lightStates.size(), 1u);
  EXPECT_EQ(lightStates["sun"].Pose(),
      ignition::math::Pose3d(10, 20, 20, 0, 0, 0));
  EXPECT_EQ(lightStates["fake_light"].Pose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  // Check addition
  worldState2 = worldState1 + worldState0;
  EXPECT_FALSE(worldState2.IsZero());

  // Check there were no deletions or insertions
  deletions = worldState2.Deletions();
  EXPECT_EQ(deletions.size(), 0u);

  insertions = worldState2.Insertions();
  EXPECT_EQ(insertions.size(), 0u);

  // Check that all entities are present
  EXPECT_EQ(worldState2.GetModelStateCount(), 4u);
  EXPECT_EQ(worldState2.LightStateCount(), 1u);

  modelStates = worldState2.GetModelStates();
  EXPECT_EQ(modelStates.size(), 4u);
  EXPECT_EQ(modelStates["ground_plane"].GetPose(),
      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  EXPECT_EQ(modelStates["box"].GetPose(),
      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));
  EXPECT_EQ(modelStates["sphere"].GetPose(),
      ignition::math::Pose3d(2, 5.5, 6.5, 0, 0, 0));
  EXPECT_EQ(modelStates["cylinder"].GetPose().pos,
      ignition::math::Vector3d(3, 4.5, 9.5));

  lightStates = worldState2.LightStates();
  EXPECT_EQ(lightStates.size(), 1u);
  EXPECT_EQ(lightStates["sun"].Pose(),
      ignition::math::Pose3d(10, 20, 40, 0, 0, 0));

  // Copy by assignment
  worldState1 = worldState0;

  // Check states are equal
  EXPECT_EQ(worldState0.GetName(), worldState1.GetName());
  EXPECT_TRUE((worldState0 - worldState1).IsZero());
}

//////////////////////////////////////////////////
TEST_F(WorldStateTest, Times)
{
  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create the world state
  physics::WorldState worldState(world);

  // Check default times
  EXPECT_EQ(worldState.GetSimTime(), common::Time(0));
  EXPECT_GT(worldState.GetWallTime(), common::Time(0));
  EXPECT_EQ(worldState.GetRealTime(), common::Time(0));

  // Set times
  worldState.SetSimTime(common::Time(1));
  worldState.SetWallTime(common::Time(2));
  worldState.SetRealTime(common::Time(3));

  // Check times
  EXPECT_EQ(worldState.GetSimTime(), common::Time(1));
  EXPECT_EQ(worldState.GetWallTime(), common::Time(2));
  EXPECT_EQ(worldState.GetRealTime(), common::Time(3));
}
