/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "physics/physics.hh"

using namespace gazebo;
class StateTest : public ServerFixture
{
};

TEST_F(StateTest, WorldStateNoWorld)
{
  common::Time wallTime = common::Time::GetWallTime();
  physics::WorldState state;

  // Check that the time are correct.
  EXPECT_LE(wallTime, state.GetWallTime());
  EXPECT_EQ(wallTime.sec, state.GetWallTime().sec);
  EXPECT_LT(state.GetWallTime().nsec - wallTime.nsec, 10000);
  EXPECT_EQ(state.GetRealTime(), common::Time());
  EXPECT_EQ(state.GetSimTime(), common::Time());

  // The default name is empty.
  EXPECT_TRUE(state.GetName().empty());

  // Set SetName
  state.SetName("my_world_state_name");
  EXPECT_EQ(state.GetName(), "my_world_state_name");

  // Test operator=
  physics::WorldState state2 = state;
  EXPECT_EQ(state2.GetName(), "my_world_state_name");
  EXPECT_LE(wallTime, state2.GetWallTime());
  EXPECT_EQ(state.GetWallTime(), state2.GetWallTime());
  EXPECT_EQ(state2.GetRealTime(), common::Time());
  EXPECT_EQ(state2.GetSimTime(), common::Time());

  // Test operator-
  state2 = state2 - state;
  EXPECT_EQ(state2.GetWallTime(), common::Time());
  EXPECT_EQ(state2.GetRealTime(), common::Time());
  EXPECT_EQ(state2.GetSimTime(), common::Time());
}

TEST_F(StateTest, WorldStateEmptyWorld)
{
  std::vector<physics::ModelState> modelStates;

  Load("worlds/empty.world");
  physics::WorldState state(physics::get_world("default"));
  EXPECT_EQ(state.GetSimTime(), common::Time());

  // Test WorldState::GetModelStates and WorldState::GetModelStateCount
  {
    modelStates = state.GetModelStates();
    EXPECT_EQ(static_cast<int>(modelStates.size()), 1);
    EXPECT_EQ(static_cast<int>(state.GetModelStateCount()), 1);
  }

  // Test WorldState::GetModelState
  {
    EXPECT_EQ(state.GetModelState(0), modelStates[0]);
    EXPECT_EQ(state.GetModelState("ground_plane"), modelStates[0]);

    EXPECT_THROW(state.GetModelState("bad_model_name"), common::Exception);
    EXPECT_EQ(state.GetModelState(2), physics::ModelState());
  }

  // Test WorldState::HasModelState
  {
    EXPECT_TRUE(state.HasModelState("ground_plane"));
    EXPECT_FALSE(state.HasModelState("bad_name"));
  }

  // Test WorldState::IsZero
  {
    EXPECT_TRUE(state.IsZero());
  }

  // Test WorldState::operator= adn WorldState::operator-
  {
    physics::WorldState newState = state;
    EXPECT_EQ(newState, state);

    newState = newState - state;
    EXPECT_EQ(newState.GetSimTime(), common::Time());
    EXPECT_EQ(newState.GetRealTime(), common::Time());
    EXPECT_EQ(newState.GetWallTime(), common::Time());

    EXPECT_EQ(static_cast<int>(state.GetModelStateCount()), 0);
  }

}

TEST_F(StateTest, State)
{
  /// \TODO: Redo state test
  /*
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  physics::WorldState worldState = world->GetState();
  physics::ModelState modelState = worldState.GetModelState(0);
  physics::LinkState linkState = modelState.GetLinkState(0);
  physics::CollisionState collisionState = linkState.GetCollisionState(0);

  math::Pose pose;
  EXPECT_EQ(static_cast<unsigned int>(1), worldState.GetModelStateCount());
  EXPECT_EQ(static_cast<unsigned int>(1), modelState.GetLinkStateCount());
  EXPECT_EQ(static_cast<unsigned int>(1), linkState.GetCollisionStateCount());
  EXPECT_EQ(pose, modelState.GetPose());
  EXPECT_EQ(pose, linkState.GetPose());
  EXPECT_EQ(pose, collisionState.GetPose());

  Unload();
  Load("worlds/shapes.world");
  world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);
  worldState = world->GetState();

  for (unsigned int i = 0; i < worldState.GetModelStateCount(); ++i)
  {
    modelState = worldState.GetModelState(i);
    if (modelState.GetName() == "plane")
      pose.Set(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "box")
      pose.Set(math::Vector3(0, 0, 0.5), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "sphere")
      pose.Set(math::Vector3(0, 1.5, 0.5), math::Quaternion(0, 0, 0));
    else if (modelState.GetName() == "cylinder")
      pose.Set(math::Vector3(0, -1.5, 0.5), math::Quaternion(0, 0, 0));

    EXPECT_TRUE(pose == modelState.GetPose());
  }

  // Move the box
  world->GetModel("box")->SetWorldPose(
      math::Pose(math::Vector3(1, 2, 0.5), math::Quaternion(0, 0, 0)));

  gazebo::common::Time::MSleep(10);

  // Make sure the box has been moved
  physics::ModelState modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(1, 2, 0.5), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());

  // Reset world state, and check for correctness
  world->SetState(worldState);
  modelState2 = world->GetState().GetModelState("box");
  pose.Set(math::Vector3(0, 0, 0.5), math::Quaternion(0, 0, 0));
  EXPECT_TRUE(pose == modelState2.GetPose());
  Unload();
  */
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
