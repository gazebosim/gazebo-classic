/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include "test/integration/joint_test.hh"

using namespace gazebo;

/// \brief Test removing different worlds.
class WorldRemoveTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Test removing a blank world.
  /// \param[in] _physicsEngines Physics engine to be tested.
  public: void RemoveBlankWorld(const std::string &_physicsEngine);

  /// \brief Test removing a world which contains models and lights, but no
  /// joints.
  /// \param[in] _physicsEngines Physics engine to be tested.
  public: void RemoveWorldWithEntities(const std::string &_physicsEngine);
};

/// \brief Test removing worlds with joints. Inherits from JointTest to make
/// use of SpawnJoint.
class WorldRemoveJointsTest : public JointTest
{
  /// \brief Test removing worlds which contain joints.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to spawn and test.
  public: void RemoveWorldWithJoint(const std::string &_physicsEngine,
                                    const std::string &_jointType);
};

/////////////////////////////////////////////////
unsigned int WorldTopicCount(std::map<std::string, std::list<std::string>>
    &_msgTypes)
{
  unsigned int count = 0;
  for (auto msgType : _msgTypes)
  {
    for (auto topic : msgType.second)
    {
      if (topic.find("/gazebo/default") != std::string::npos)
      {
        count++;
      }
    }
  }
  return count;
}

/////////////////////////////////////////////////
void WorldRemoveTest::RemoveBlankWorld(const std::string &_physicsEngine)
{
  // Load a blank world
  this->Load("worlds/blank.world", false, _physicsEngine);

  // Give time for everything to be created
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are worlds running
  EXPECT_TRUE(physics::worlds_running());

  // Get world pointer
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto worldPtrCount = world.use_count();
  EXPECT_GT(worldPtrCount, 1);

  // Get physics engine pointer
  auto physicsEngine = world->GetPhysicsEngine();
  ASSERT_TRUE(physicsEngine != nullptr);

  auto physicsEnginePtrCount = physicsEngine.use_count();
  EXPECT_GT(physicsEnginePtrCount, 1);

  // Check advertised topics
  auto msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  auto worldTopicCount = WorldTopicCount(msgTypes);
  EXPECT_GT(worldTopicCount, 0u);

  // Stats before removing world
  gzmsg << "Stats before removing world:" << std::endl
        << "- WorldPtr use count: [" << world.use_count() << "]" << std::endl
        << "- PhysicsEnginePtr use count: [" << physicsEngine.use_count() << "]"
        << std::endl << "- Topics in this world: [" << worldTopicCount << "]"
        << std::endl;

  // Remove world
  physics::remove_worlds();

  // Give time for everything to be removed
  sleep = 0;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are no worlds running
  EXPECT_FALSE(physics::worlds_running());

  // Check the only shared pointer left to the physics engine is this one
  EXPECT_LT(physicsEngine.use_count(), physicsEnginePtrCount);
  EXPECT_EQ(physicsEngine.use_count(), 1);

  // Release the last physics engine pointer
  physicsEngine.reset();

  // Check the only pointer left to the world is this one
  EXPECT_LT(world.use_count(), worldPtrCount);
  EXPECT_EQ(world.use_count(), 1);

  // Release the last world pointer
  world.reset();

  // Check we can't get the world pointer
  gzmsg << "Expect exception when trying to get removed world:" << std::endl;
  EXPECT_THROW(world = physics::get_world("default"), common::Exception);
  EXPECT_TRUE(world == nullptr);

  // Check all topics related to that world are gone
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_LT(WorldTopicCount(msgTypes), worldTopicCount);
  EXPECT_EQ(WorldTopicCount(msgTypes), 0u);

  // Stats after removing world
  gzmsg << "Stats after removing world:" << std::endl
        << "- WorldPtr use count: [" << world.use_count() << "]" << std::endl
        << "- PhysicsEnginePtr use count: [" << physicsEngine.use_count() << "]"
        << std::endl << "- Topics in this world: [" <<
        WorldTopicCount(msgTypes) << "]" << std::endl;
}

/////////////////////////////////////////////////
void WorldRemoveTest::RemoveWorldWithEntities(const std::string &_physicsEngine)
{
  // Load a world with entities
  this->Load("worlds/shapes.world", false, _physicsEngine);

  // Give time for everything to be created
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are worlds running
  EXPECT_TRUE(physics::worlds_running());

  // Get world pointer
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto worldPtrCount = world.use_count();
  EXPECT_GT(worldPtrCount, 1);

  // Get physics engine pointer
  auto physicsEngine = world->GetPhysicsEngine();
  ASSERT_TRUE(physicsEngine != nullptr);

  auto physicsEnginePtrCount = physicsEngine.use_count();
  EXPECT_GT(physicsEnginePtrCount, 1);

  // Check advertised topics
  auto msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  auto worldTopicCount = WorldTopicCount(msgTypes);
  EXPECT_GT(worldTopicCount, 0u);

  // Get model pointers
  std::vector<std::string> modelNames;
  modelNames.push_back("ground_plane");
  modelNames.push_back("box");
  modelNames.push_back("sphere");
  modelNames.push_back("cylinder");

  std::vector<physics::ModelPtr> modelPtrs;
  for (auto &name : modelNames)
  {
    auto model = world->GetModel(name);
    ASSERT_TRUE(model != nullptr);
    modelPtrs.push_back(model);
  }

  // Get light pointers
  std::vector<std::string> lightNames;
  lightNames.push_back("sun");

  std::vector<physics::LightPtr> lightPtrs;
  for (auto &name : lightNames)
  {
    auto light = world->Light(name);
    ASSERT_TRUE(light != nullptr);
    lightPtrs.push_back(light);
  }

  // Stats before removing world
  gzmsg << "Stats before removing world:" << std::endl;
  gzmsg << "- [WorldPtr] use count: [" << world.use_count() << "]"
        << std::endl;
  gzmsg << "- [PhysicsEnginePtr] use count: [" << physicsEngine.use_count()
        << "]" << std::endl;
  gzmsg << "- Topics in this world: [" << worldTopicCount << "]"
        << std::endl;

  for (size_t i = 0; i < modelNames.size(); ++i)
  {
    gzmsg << "- [" << modelNames[i] << "] ptr use count: ["
        << modelPtrs[i].use_count() << "]" << std::endl;
  }

  for (size_t i = 0; i < lightNames.size(); ++i)
  {
    gzmsg << "- [" << lightNames[i] << "] ptr use count: ["
        << lightPtrs[i].use_count() << "]" << std::endl;
  }

  // Remove world
  physics::remove_worlds();

  // Give time for everything to be removed
  sleep = 0;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are no worlds running
  EXPECT_FALSE(physics::worlds_running());

  // Check the only shared pointers to entities left are the ones we're holding
  for (auto &ptr : modelPtrs)
  {
    EXPECT_EQ(ptr.use_count(), 1);
  }
  for (auto &ptr : lightPtrs)
  {
    EXPECT_EQ(ptr.use_count(), 1);
  }

  // Release entity pointers
  for (auto &ptr : modelPtrs)
  {
    ptr.reset();
  }
  for (auto &ptr : lightPtrs)
  {
    ptr.reset();
  }

  // Check the only shared pointer left to the physics engine is this one
  EXPECT_LT(physicsEngine.use_count(), physicsEnginePtrCount);
  EXPECT_EQ(physicsEngine.use_count(), 1);

  // Release the last physics engine pointer
  physicsEngine.reset();

  // Check the only pointer left to the world is this one
  EXPECT_LT(world.use_count(), worldPtrCount);
  EXPECT_EQ(world.use_count(), 1);

  // Release the last world pointer
  world.reset();

  // Check we can't get the world pointer
  gzmsg << "Expect exception when trying to get removed world:" << std::endl;
  EXPECT_THROW(world = physics::get_world("default"), common::Exception);
  EXPECT_TRUE(world == nullptr);

  // Check all topics related to that world are gone
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_LT(WorldTopicCount(msgTypes), worldTopicCount);
  EXPECT_EQ(WorldTopicCount(msgTypes), 0u);

  // Stats after removing world
  gzmsg << "Stats after removing world:" << std::endl;
  gzmsg << "- WorldPtr use count: [" << world.use_count() << "]"
        << std::endl;
  gzmsg << "- PhysicsEnginePtr use count: [" << physicsEngine.use_count() << "]"
        << std::endl;
  gzmsg << "- Topics in this world: [" << WorldTopicCount(msgTypes) << "]"
        << std::endl;

  for (size_t i = 0; i < modelNames.size(); ++i)
  {
    gzmsg << "- [" << modelNames[i] << "] ptr use count: ["
        << modelPtrs[i].use_count() << "]" << std::endl;
  }

  for (size_t i = 0; i < lightNames.size(); ++i)
  {
    gzmsg << "- [" << lightNames[i] << "] ptr use count: ["
        << lightPtrs[i].use_count() << "]" << std::endl;
  }
}

/////////////////////////////////////////////////
TEST_P(WorldRemoveTest, RemoveBlankWorld)
{
  RemoveBlankWorld(GetParam());
}

/////////////////////////////////////////////////
TEST_P(WorldRemoveTest, RemoveWorldWithEntities)
{
  RemoveWorldWithEntities(GetParam());
}

INSTANTIATE_TEST_CASE_P(WorldRemoveTest, WorldRemoveTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
void WorldRemoveJointsTest::RemoveWorldWithJoint(
    const std::string &_physicsEngine, const std::string &_jointType)
{
  gzmsg << "Test joint [" << _jointType << "] engine [" << _physicsEngine
       << "]" << std::endl;

  // Load an empty world
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Give time for everything to be created
  int sleep = 0;
  int maxSleep = 10;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are worlds running
  EXPECT_TRUE(physics::worlds_running());

  // Spawn a model with a joint
  auto joint = SpawnJoint(_jointType, false, false);
  ASSERT_TRUE(joint != nullptr);

  // Get world pointer
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  auto worldPtrCount = world.use_count();
  EXPECT_GT(worldPtrCount, 1);

  // Get model pointer
  auto model = world->GetModel("joint_model0");
  ASSERT_TRUE(model != nullptr);

  // Check model has the joint
  EXPECT_EQ(model->GetJointCount(), 1u);
  EXPECT_EQ(joint, model->GetJoint("joint"));

  // Get link pointers
  auto parentLink = model->GetLink("parent");
  ASSERT_TRUE(parentLink != nullptr);

  auto childLink = model->GetLink("child");
  ASSERT_TRUE(childLink != nullptr);

  // Get physics engine pointer
  auto physicsEngine = world->GetPhysicsEngine();
  ASSERT_TRUE(physicsEngine != nullptr);

  auto physicsEnginePtrCount = physicsEngine.use_count();
  EXPECT_GT(physicsEnginePtrCount, 1);

  // Check advertised topics
  auto msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_FALSE(msgTypes.empty());

  auto worldTopicCount = WorldTopicCount(msgTypes);
  EXPECT_GT(worldTopicCount, 0u);

  // Stats before removing world
  gzmsg << "Stats before removing world:" << std::endl
        << "- WorldPtr use count: [" << world.use_count() << "]"
        << std::endl
        << "- PhysicsEnginePtr use count: [" << physicsEngine.use_count() << "]"
        << std::endl
        << "- Topics in this world: [" << worldTopicCount << "]"
        << std::endl
        << "- model ptr use count: [" << model.use_count() << "]"
        << std::endl
        << "- parent link ptr use count: [" << parentLink.use_count() << "]"
        << std::endl
        << "- childLink ptr use count: [" << childLink.use_count() << "]"
        << std::endl
        << "- joint ptr use count: [" << joint.use_count() << "]"
        << std::endl;

  // Remove world
  physics::remove_worlds();

  // Give time for everything to be removed
  sleep = 0;
  while (sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(300);
    sleep++;
  }

  // Check there are no worlds running
  EXPECT_FALSE(physics::worlds_running());

  // Check the only shared pointers left are these
  EXPECT_EQ(model.use_count(), 1) << "Model pointer [" << model << "]";
  EXPECT_EQ(parentLink.use_count(), 1)
      << "Parent link pointer [" << parentLink << "]";
  EXPECT_EQ(childLink.use_count(), 1)
      << "Child link pointer [" << childLink << "]";
  EXPECT_EQ(joint.use_count(), 1) << "Joint pointer [" << joint << "]";

  // Release pointers
  model.reset();
  parentLink.reset();
  childLink.reset();
  joint.reset();

  // Check the only shared pointer left to the physics engine is this one
  EXPECT_LT(physicsEngine.use_count(), physicsEnginePtrCount);
  EXPECT_EQ(physicsEngine.use_count(), 1);

  // Release the last physics engine pointer
  physicsEngine.reset();

  // Check the only pointer left to the world is this one
  EXPECT_LT(world.use_count(), worldPtrCount);
  EXPECT_EQ(world.use_count(), 1);

  // Release the last world pointer
  world.reset();

  // Check we can't get the world pointer
  gzmsg << "Expect exception when trying to get removed world:" << std::endl;
  EXPECT_THROW(world = physics::get_world("default"), common::Exception);
  EXPECT_TRUE(world == nullptr);

  // Check all topics related to that world are gone
  msgTypes = gazebo::transport::getAdvertisedTopics();
  EXPECT_LT(WorldTopicCount(msgTypes), worldTopicCount);
  EXPECT_EQ(WorldTopicCount(msgTypes), 0u);

  // Stats after removing world
  gzmsg << "Stats after removing world:" << std::endl
        << "- WorldPtr use count: [" << world.use_count() << "]"
        << std::endl
        << "- PhysicsEnginePtr use count: [" << physicsEngine.use_count() << "]"
        << std::endl
        << "- Topics in this world: [" << WorldTopicCount(msgTypes) << "]"
        << std::endl
        << "- model ptr use count: [" << model.use_count() << "]"
        << std::endl
        << "- joint ptr use count: [" << joint.use_count() << "]"
        << std::endl;
}

///////////////////////////////////////////////////
TEST_P(WorldRemoveJointsTest, RemoveWorldWithJoint)
{
  if (this->jointType == "gearbox" && this->physicsEngine != "ode")
  {
    gzerr << "Skip test, gearbox is only supported in ODE. " <<
         "See issues: #859, #1914, #1915" << std::endl;
    return;
  }
  if (this->physicsEngine == "simbody" && this->jointType == "revolute2")
  {
    gzerr << "Skip test, revolute2 not supported in simbody, see issue #859."
          << std::endl;
    return;
  }
  RemoveWorldWithJoint(this->physicsEngine, this->jointType);
}

INSTANTIATE_TEST_CASE_P(RemoveJointTypes, WorldRemoveJointsTest,
  ::testing::Combine(PHYSICS_ENGINE_VALUES,
  ::testing::Values("revolute"
                  , "prismatic"
                  , "screw"
                  , "universal"
                  , "fixed"
                  , "ball"
                  , "revolute2"
                  , "gearbox")));

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

