/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include <string.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

typedef std::tr1::tuple<const char *, const char *, int> string2_int;

class WorldResetTest : public ServerFixture,
                       public ::testing::WithParamInterface<string2_int>
{
  /// \brief Test to see if model pose is reset when the world is reset
  /// \param[in] _physicsEngine Physics engine type.
  /// \param[in] _world Name of world to load
  /// \param[in] _resets Number of resets to perform in the test
  public: void ModelPose(const std::string &_physicsEngine,
                         const std::string &_world, const int _resets);

  /// \brief Test to see if nested model pose is reset when the world is reset
  /// \param[in] _physicsEngine Physics engine type.
  /// \param[in] _world Name of world to load
  /// \param[in] _resets Number of resets to perform in the test
  public: void NestedModelPose(const std::string &_physicsEngine,
                         const std::string &_world, const int _resets);

  /// \brief Test resetting different worlds
  /// \param[in] _physicsEngine Physics engine type.
  /// \param[in] _world Name of world to load
  /// \param[in] _resets Number of resets to perform in the test
  public: void WorldName(const std::string &_physicsEngine,
                         const std::string &_world, const int _resets);
};

class WorldControlResetTest : public ServerFixture,
                       public ::testing::WithParamInterface<const char *>
{
  /// \brief Test to see if model pose is reset when the world is reset through
  /// world_control topic
  /// \param[in] _physicsEngine Physics engine type.
  public: void ModelOverlapReset(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void WorldControlResetTest::ModelOverlapReset(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody fails this test due to issue #1957" << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, dt*steps, dt);

  // spawn a box with known initial pose
  ignition::math::Pose3d initialPose(1, 2, 0.0, 0, 0, 1.57);
  math::Vector3 size(1, 1, 1);
  SpawnBox("box", size, initialPose.Pos(), initialPose.Rot().Euler(), false);
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);

  // spawn another box that overlaps with the previous box
  ignition::math::Pose3d initialPose2(1.2, 2, 0.5, 0, 0, 0);
  SpawnBox("box2", size, initialPose2.Pos(), initialPose2.Rot().Euler(), false);
  physics::ModelPtr model2 = world->GetModel("box2");
  ASSERT_TRUE(model2 != NULL);

  // physics engine has not stepped yet since the boxes were spawned so they
  // should stay overlapped. Verify pose
  EXPECT_EQ(model->GetWorldPose(), initialPose);
  EXPECT_EQ(model2->GetWorldPose(), initialPose2);

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime2 = world->GetSimTime().Double();
  EXPECT_NEAR(simTime2, simTime + dt*steps, dt);

  // physics engine has stepped, verify the models are pushed apart
  // Note: DART doesn't really mind overlapping models
  if (_physicsEngine != "dart")
  {
    EXPECT_NE(model->GetWorldPose(), initialPose);
    EXPECT_NE(model2->GetWorldPose(), initialPose2);
  }

  // Create a publisher to reset gzserver
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::WorldControl>(
        "/gazebo/default/world_control");
  pub->WaitForConnection();

  // Reset world, verify time == 0
  gazebo::msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  pub->Publish(msg);

  int sleep = 0;
  int maxSleep = 20;
  // Wait for sim time to be reset
  while (sleep < maxSleep)
  {
    simTime = world->GetSimTime().Double();
    if (ignition::math::equal(simTime, 0.0, dt))
      break;
    sleep++;
    gazebo::common::Time::MSleep(100);
  }

  simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, 0.0, dt);

  // verify boxes have moved back to initial pose and they are not pushed
  // apart
  EXPECT_EQ(model->GetWorldPose(), initialPose);
  EXPECT_EQ(model2->GetWorldPose(), initialPose2);
}

/////////////////////////////////////////////////
TEST_P(WorldControlResetTest, ModelOverlapReset)
{
  ModelOverlapReset(GetParam());
}

/////////////////////////////////////////////////
void WorldResetTest::ModelPose(const std::string &_physicsEngine,
                               const std::string &_world, const int _resets)
{
  if (_physicsEngine == "simbody" &&
      _world.find("pr2") != std::string::npos)
  {
    gzerr << "Simbody fails this test with the PR2 due to issue #1672"
          << std::endl;
    return;
  }
  if (_physicsEngine == "simbody" &&
      _world.find("nested_model") != std::string::npos)
  {
    gzerr << "Simbody fails this test with nested models due to issue #1718"
          << std::endl;
    return;
  }
  if (_physicsEngine == "dart" &&
      _world.find("pr2") != std::string::npos)
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  Load(_world, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, dt*steps, dt);

  ignition::math::Pose3d initialPose(1, 2, 0.5, 0, 0, 1.57);

  // spawn a box with known initial pose
  math::Vector3 size(1, 1, 1);
  SpawnBox("box", size, initialPose.Pos(), initialPose.Rot().Euler(), false);
  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);

  // verify box pose
  EXPECT_EQ(model->GetWorldPose(), initialPose);

  // move box to new pose
  ignition::math::Pose3d newPose(4, 5, 0.5, 0, 0, 0);
  model->SetWorldPose(newPose);
  EXPECT_EQ(model->GetWorldPose(), newPose);

  // Reset world repeatedly
  for (int i = 0; i < _resets; ++i)
  {
    // Reset world, verify time == 0
    world->Reset();
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, 0.0, dt);

    // Step forward, verify time increasing
    world->Step(steps);
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, dt*steps, dt);
  }

  // verify box has moved back to initial pose
  EXPECT_EQ(model->GetWorldPose(), initialPose);
}

/////////////////////////////////////////////////
TEST_P(WorldResetTest, ModelPose)
{
  std::string physics = std::tr1::get<0>(GetParam());

  std::string worldName = std::tr1::get<1>(GetParam());
  int resets = std::tr1::get<2>(GetParam());
  gzdbg << "Physics engine [" << physics << "] "
        << "world name [" << worldName << "] "
        << "reset count [" << resets << "]"
        << std::endl;
  ModelPose(physics, worldName, resets);
}

/////////////////////////////////////////////////
void WorldResetTest::NestedModelPose(const std::string &_physicsEngine,
                                     const std::string &_world,
                                     const int _resets)
{
  if (_physicsEngine == "simbody")
  {
    gzerr  << "Nested models are not working in simbody yet, issue #1718"
        << std::endl;
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr  << "Nested models are not working in dart yet, issue #1833"
        << std::endl;
    return;
  }

  if (_world != "worlds/nested_model.world")
    return;

  Load(_world, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, dt*steps, dt);

  physics::ModelPtr model = world->GetModel("model_00");
  ASSERT_TRUE(model != NULL);

  // store all initial pose
  std::list<ignition::math::Pose3d> modelPoses;
  std::list<physics::ModelPtr> models;
  models.push_back(model);
  while (!models.empty())
  {
    physics::ModelPtr m = models.front();
    modelPoses.push_back(m->GetWorldPose().Ign());
    models.pop_front();
    for (const auto &nested : m->NestedModels())
      models.push_back(nested);
  }

  // move model to new pose
  ignition::math::Pose3d newPose(9, 5, 2.5, 0, 0, 0);
  model->SetWorldPose(newPose);
  EXPECT_EQ(model->GetWorldPose(), newPose);

  // Reset world repeatedly
  for (int i = 0; i < _resets; ++i)
  {
    // Reset world, verify time == 0
    world->Reset();
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, 0.0, dt);

    // Step forward, verify time increasing
    world->Step(steps);
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, dt*steps, dt);

    // verify all nested models have moved back to initial pose
    models.clear();
    models.push_back(model);
    auto modelPosesCopy = modelPoses;
    while (!models.empty())
    {
      physics::ModelPtr m = models.front();
      EXPECT_EQ(m->GetWorldPose(), modelPosesCopy.front());
      models.pop_front();
      modelPosesCopy.pop_front();
      for (const auto &nested : m->NestedModels())
        models.push_back(nested);
    }
  }
}

/////////////////////////////////////////////////
TEST_P(WorldResetTest, NestedModelPose)
{
  std::string physics = std::tr1::get<0>(GetParam());

  std::string worldName = std::tr1::get<1>(GetParam());
  int resets = std::tr1::get<2>(GetParam());
  gzdbg << "Physics engine [" << physics << "] "
        << "world name [" << worldName << "] "
        << "reset count [" << resets << "]"
        << std::endl;
  NestedModelPose(physics, worldName, resets);
}

/////////////////////////////////////////////////
void WorldResetTest::WorldName(const std::string &_physicsEngine,
                               const std::string &_world, const int _resets)
{
  if (_physicsEngine == "simbody" &&
      _world.find("pr2") != std::string::npos)
  {
    gzerr << "Simbody fails this test with the PR2 due to issue #1672"
          << std::endl;
    return;
  }
  if (_physicsEngine == "simbody" &&
      _world.find("nested_model") != std::string::npos)
  {
    gzerr << "Simbody fails this test with nested models due to issue #1718"
          << std::endl;
    return;
  }
  if (_physicsEngine == "dart" &&
      _world.find("pr2") != std::string::npos)
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  Load(_world, true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  double dt = physics->GetMaxStepSize();
  unsigned int steps = 250;

  // Step forward, verify time increasing
  world->Step(steps);
  double simTime = world->GetSimTime().Double();
  EXPECT_NEAR(simTime, dt*steps, dt);

  // Reset world repeatedly
  for (int i = 0; i < _resets; ++i)
  {
    // Reset world, verify time == 0
    world->Reset();
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, 0.0, dt);

    // Step forward, verify time increasing
    world->Step(steps);
    simTime = world->GetSimTime().Double();
    EXPECT_NEAR(simTime, dt*steps, dt);
  }
}

/////////////////////////////////////////////////
TEST_P(WorldResetTest, WorldName)
{
  std::string physics = std::tr1::get<0>(GetParam());
  std::string worldName = std::tr1::get<1>(GetParam());
  int resets = std::tr1::get<2>(GetParam());
  gzdbg << "Physics engine [" << physics << "] "
        << "world name [" << worldName << "] "
        << "reset count [" << resets << "]"
        << std::endl;
  WorldName(physics, worldName, resets);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldControlResetTest,
    PHYSICS_ENGINE_VALUES);

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldResetTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("worlds/empty.world",
                    "worlds/pr2.world",
                    "worlds/nested_model.world"),
  ::testing::Range(1, 3)));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
