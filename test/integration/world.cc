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
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class WorldTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  /// \brief Test clearing an empty (blank) world.
  /// \param[in] _physicsEngine Name of physics engine.
  public: void ClearEmptyWorld(const std::string &_physicsEngine);

  /// \brief Test Gravity, SetGravity
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Gravity(const std::string &_physicsEngine);

  /// \brief Test MagneticField, SetMagneticField
  /// \param[in] _physicsEngine Physics engine to use.
  public: void MagneticField(const std::string &_physicsEngine);
};

/// \brief Pose after physics update
ignition::math::Pose3d g_poseAfterUpdate;

/// \brief Pose before physics update
ignition::math::Pose3d g_poseBeforeUpdate;

/// \brief Has the WorldUpdateBegin event been called
bool g_updateBeginCalled = false;

/// \brief Has the BeforePhysicsUpdate event been called
bool g_beforePhysicsUpdateCalled = false;

/// \brief Has the WorldUpdateEnd event been called
bool g_updateEndCalled = false;

/// \brief Callback for WorldUpdateBegin event, just records it's been called.
/// \param[in] _updateInfo Information about the event time and world.
void onWorldUpdateBegin(const common::UpdateInfo & /*_updateInfo*/)
{
  g_updateBeginCalled = true;
}

/// \brief Callback for BeforePhysicsUpdate event.
/// Record that it has been called, and also record the reported ball
/// position.
/// \param[in] updateInfo Information about the event time and world.
void beforePhysicsUpdate(const common::UpdateInfo &_updateInfo)
{
  g_beforePhysicsUpdateCalled = true;

  physics::WorldPtr world = physics::get_world(_updateInfo.worldName);
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr sphereModel = world->GetModel("sphere");
  ASSERT_TRUE(sphereModel != NULL);

  physics::LinkPtr link = sphereModel->GetLink("link");
  ASSERT_TRUE(link != NULL);

  g_poseBeforeUpdate = link->GetWorldPose().Ign();
}

/// \brief Callback for WorldUpdateEnd event, just records it's been called.
void onWorldUpdateEnd()
{
  g_updateEndCalled = true;
}

/////////////////////////////////////////////////
void WorldTest::ClearEmptyWorld(const std::string &_physicsEngine)
{
  this->Load("worlds/blank.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  gzdbg << "Engine: " << _physicsEngine << std::endl;
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  EXPECT_EQ(world->GetModelCount(), 0u);

  world->Clear();

  // Wait some bit of time since World::Clear is not immediate.
  for (unsigned int i = 0; i < 20; ++i)
    common::Time::MSleep(500);

  EXPECT_EQ(world->GetModelCount(), 0u);

  // Now spawn something, and the model count should increase
  SpawnSphere("sphere", math::Vector3(0, 0, 1), math::Vector3(0, 0, 0));
  EXPECT_EQ(world->GetModelCount(), 1u);
}

/////////////////////////////////////////////////
TEST_P(WorldTest, ClearEmptyWorld)
{
  ClearEmptyWorld(GetParam());
}

/////////////////////////////////////////////////
TEST_F(WorldTest, Clear)
{
  Load("worlds/pioneer2dx.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  EXPECT_EQ(world->GetModelCount(), 2u);

  world->Clear();
  while (world->GetModelCount() > 0u)
    common::Time::MSleep(1000);

  EXPECT_EQ(world->GetModelCount(), 0u);

  SpawnSphere("sphere", math::Vector3(0, 0, 1), math::Vector3(0, 0, 0));

  EXPECT_EQ(world->GetModelCount(), 1u);
}

/////////////////////////////////////////////////
void WorldTest::Gravity(const std::string &_physicsEngine)
{
  this->Load("worlds/friction_demo.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  auto physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  double dt = physics->GetMaxStepSize();

  // expect value from world file
  EXPECT_EQ(ignition::math::Vector3d(0, -1, -1), world->Gravity());

  // set gravity to point up
  const ignition::math::Vector3d gravity(0, 0, 10);
  world->SetGravity(gravity);
  EXPECT_EQ(gravity, world->Gravity());

  // get initial pose and velocity of a box
  auto model = world->GetModel("box_01_model");
  ASSERT_TRUE(model != NULL);
  auto initialPose = model->GetWorldPose().Ign();
  auto initialVelocity = model->GetWorldLinearVel().Ign();
  EXPECT_EQ(ignition::math::Vector3d::Zero, initialVelocity);

  // step simulation and verify that box moves upwards
  const int steps = 100;
  world->Step(steps);
  auto expectedVelocity = dt * steps * gravity;
  auto velocity = model->GetWorldLinearVel().Ign();
  auto expectedPosition = initialPose.Pos() + 0.5*(dt*steps) * expectedVelocity;
  EXPECT_GT(velocity.Z(), 0.95*expectedVelocity.Z());
  EXPECT_GT(model->GetWorldPose().Ign().Pos().Z(), 0.95*expectedPosition.Z());

  // set gravity back to zero
  world->SetGravity(ignition::math::Vector3d::Zero);
  EXPECT_EQ(ignition::math::Vector3d::Zero, world->Gravity());

  // step simulation and verify that velocity stays constant
  world->Step(steps);
  EXPECT_EQ(velocity, model->GetWorldLinearVel().Ign());
}

/////////////////////////////////////////////////
TEST_P(WorldTest, Gravity)
{
  Gravity(GetParam());
}

/////////////////////////////////////////////////
void WorldTest::MagneticField(const std::string &_physicsEngine)
{
  this->Load("worlds/magnetometer.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // expect value from world file
  EXPECT_EQ(ignition::math::Vector3d(1, 0, 0), world->MagneticField());

  // set magnetic field to zero
  world->SetMagneticField(ignition::math::Vector3d::Zero);
  EXPECT_EQ(ignition::math::Vector3d::Zero, world->MagneticField());

  // set magnetic field to ones
  world->SetMagneticField(ignition::math::Vector3d::One);
  EXPECT_EQ(ignition::math::Vector3d::One, world->MagneticField());
}

/////////////////////////////////////////////////
TEST_P(WorldTest, MagneticField)
{
  MagneticField(GetParam());
}

/////////////////////////////////////////////////
TEST_F(WorldTest, ModifyLight)
{
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->SetPaused(true);

  // Make sure there is only one light, and it is named "sun"
  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 1u);
    EXPECT_STREQ(lights[0]->GetName().c_str(), "sun");

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 1);
    EXPECT_STREQ(sceneMsg.light(0).name().c_str(), "sun");
  }

  transport::PublisherPtr lightModifyPub = this->node->Advertise<msgs::Light>(
        "~/light/modify");

  // Set the light to be green
  {
    msgs::Light lightMsg;
    lightMsg.set_name("sun");
    msgs::Set(lightMsg.mutable_diffuse(), common::Color(0, 1, 0));
    lightModifyPub->Publish(lightMsg);
  }

  // Allow the world time to process the messages
  // Must be big enough to pass `processMsgsPeriod`
  world->Step(1000);

  // Get the new scene, and make sure the color of the "sun" light is
  // correct.
  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 1u);
    EXPECT_STREQ(lights[0]->GetName().c_str(), "sun");
    msgs::Light lightMsg;
    lights[0]->FillMsg(lightMsg);
    EXPECT_EQ(lightMsg.diffuse().r(), 0);
    EXPECT_EQ(lightMsg.diffuse().g(), 1);
    EXPECT_EQ(lightMsg.diffuse().b(), 0);

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 1);
    EXPECT_STREQ(sceneMsg.light(0).name().c_str(), "sun");
    EXPECT_EQ(sceneMsg.light(0).diffuse().r(), 0);
    EXPECT_EQ(sceneMsg.light(0).diffuse().g(), 1);
    EXPECT_EQ(sceneMsg.light(0).diffuse().b(), 0);
  }

  transport::PublisherPtr lightFactoryPub = this->node->Advertise<msgs::Light>(
        "~/factory/light");

  // Add a new light
  {
    msgs::Light lightMsg;
    lightMsg.set_name("test_light");
    msgs::Set(lightMsg.mutable_diffuse(), common::Color(1, 0, 1));
    lightMsg.set_type(msgs::Light::POINT);
    lightFactoryPub->Publish(lightMsg);
  }

  // Allow the world time to process the messages
  world->Step(1000);

  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 2u);
    EXPECT_STREQ(lights[1]->GetName().c_str(), "test_light");
    msgs::Light lightMsg;
    lights[1]->FillMsg(lightMsg);
    EXPECT_EQ(lightMsg.diffuse().r(), 1);
    EXPECT_EQ(lightMsg.diffuse().g(), 0);
    EXPECT_EQ(lightMsg.diffuse().b(), 1);
    EXPECT_EQ(lightMsg.type(), msgs::Light::POINT);

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 2);
    EXPECT_STREQ(sceneMsg.light(1).name().c_str(), "test_light");
    EXPECT_EQ(sceneMsg.light(1).diffuse().r(), 1);
    EXPECT_EQ(sceneMsg.light(1).diffuse().g(), 0);
    EXPECT_EQ(sceneMsg.light(1).diffuse().b(), 1);
    EXPECT_EQ(sceneMsg.light(1).type(), msgs::Light::POINT);
  }

  // Delete the test_light
  ServerFixture::RemoveModel("test_light");

  // Allow the world time to process the messages
  world->Step(1000);

  // Verify that the test_light is gone and that the sun remains
  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 1u);
    EXPECT_STREQ(lights[0]->GetName().c_str(), "sun");

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 1);
    EXPECT_STREQ(sceneMsg.light(0).name().c_str(), "sun");
  }

  // Add a new spot light
  {
    msgs::Light lightMsg;
    lightMsg.set_name("test_spot_light");
    msgs::Set(lightMsg.mutable_diffuse(), common::Color(1, 1, 0));
    lightMsg.set_type(msgs::Light::SPOT);
    lightFactoryPub->Publish(lightMsg);
  }

  // Allow the world time to process the messages
  world->Step(1000);

  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 2u);
    EXPECT_STREQ(lights[1]->GetName().c_str(), "test_spot_light");
    msgs::Light lightMsg;
    lights[1]->FillMsg(lightMsg);
    EXPECT_EQ(lightMsg.diffuse().r(), 1);
    EXPECT_EQ(lightMsg.diffuse().g(), 1);
    EXPECT_EQ(lightMsg.diffuse().b(), 0);
    EXPECT_EQ(lightMsg.type(), msgs::Light::SPOT);

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 2);
    EXPECT_STREQ(sceneMsg.light(1).name().c_str(), "test_spot_light");
    EXPECT_EQ(sceneMsg.light(1).diffuse().r(), 1);
    EXPECT_EQ(sceneMsg.light(1).diffuse().g(), 1);
    EXPECT_EQ(sceneMsg.light(1).diffuse().b(), 0);
    EXPECT_EQ(sceneMsg.light(1).type(), msgs::Light::SPOT);
  }

  // Modify spot light pose
  {
    msgs::Light lightMsg;
    lightMsg.set_name("test_spot_light");
    msgs::Set(lightMsg.mutable_pose(),
        ignition::math::Pose3d(
          ignition::math::Vector3d(3, 2, 1),
          ignition::math::Quaterniond(0, 1, 0, 0)));
    lightModifyPub->Publish(lightMsg);
  }

  // Allow the world time to process the messages
  world->Step(1000);

  // Verify the light gets the new pose and retains values of other properties
  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 2u);
    EXPECT_STREQ(lights[1]->GetName().c_str(), "test_spot_light");
    msgs::Light lightMsg;
    lights[1]->FillMsg(lightMsg);
    EXPECT_EQ(lightMsg.diffuse().r(), 1);
    EXPECT_EQ(lightMsg.diffuse().g(), 1);
    EXPECT_EQ(lightMsg.diffuse().b(), 0);

    EXPECT_EQ(lightMsg.pose().position().x(), 3);
    EXPECT_EQ(lightMsg.pose().position().y(), 2);
    EXPECT_EQ(lightMsg.pose().position().z(), 1);
    EXPECT_EQ(lightMsg.pose().orientation().w(), 0);
    EXPECT_EQ(lightMsg.pose().orientation().x(), 1);
    EXPECT_EQ(lightMsg.pose().orientation().y(), 0);
    EXPECT_EQ(lightMsg.pose().orientation().z(), 0);

    EXPECT_EQ(lightMsg.type(), msgs::Light::SPOT);

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 2);
    EXPECT_STREQ(sceneMsg.light(1).name().c_str(), "test_spot_light");
    EXPECT_EQ(sceneMsg.light(1).diffuse().r(), 1);
    EXPECT_EQ(sceneMsg.light(1).diffuse().g(), 1);
    EXPECT_EQ(sceneMsg.light(1).diffuse().b(), 0);

    EXPECT_EQ(sceneMsg.light(1).pose().position().x(), 3);
    EXPECT_EQ(sceneMsg.light(1).pose().position().y(), 2);
    EXPECT_EQ(sceneMsg.light(1).pose().position().z(), 1);
    EXPECT_EQ(sceneMsg.light(1).pose().orientation().w(), 0);
    EXPECT_EQ(sceneMsg.light(1).pose().orientation().x(), 1);
    EXPECT_EQ(sceneMsg.light(1).pose().orientation().y(), 0);
    EXPECT_EQ(sceneMsg.light(1).pose().orientation().z(), 0);

    EXPECT_EQ(sceneMsg.light(1).type(), msgs::Light::SPOT);
  }

  // Add a new light with the name of a light that has been deleted
  {
    msgs::Light lightMsg;
    lightMsg.set_name("test_light");
    msgs::Set(lightMsg.mutable_diffuse(), common::Color(0, 0, 1));
    lightMsg.set_type(msgs::Light::DIRECTIONAL);
    lightFactoryPub->Publish(lightMsg);
  }

  // Allow the world time to process the messages
  world->Step(1000);

  {
    // Check light objects
    physics::Light_V lights = world->Lights();
    EXPECT_EQ(lights.size(), 3u);
    EXPECT_STREQ(lights[2]->GetName().c_str(), "test_light");
    msgs::Light lightMsg;
    lights[2]->FillMsg(lightMsg);
    EXPECT_DOUBLE_EQ(lightMsg.diffuse().r(), 0);
    EXPECT_EQ(lightMsg.diffuse().g(), 0);
    EXPECT_EQ(lightMsg.diffuse().b(), 1);
    EXPECT_EQ(lightMsg.type(), msgs::Light::DIRECTIONAL);

    // Check scene message
    msgs::Scene sceneMsg = world->GetSceneMsg();
    EXPECT_EQ(sceneMsg.light_size(), 3);
    EXPECT_STREQ(sceneMsg.light(2).name().c_str(), "test_light");
    EXPECT_EQ(sceneMsg.light(2).diffuse().r(), 0);
    EXPECT_EQ(sceneMsg.light(2).diffuse().g(), 0);
    EXPECT_EQ(sceneMsg.light(2).diffuse().b(), 1);
    EXPECT_EQ(sceneMsg.light(2).type(), msgs::Light::DIRECTIONAL);
  }
}

/////////////////////////////////////////////////
TEST_F(WorldTest, RemoveModelPaused)
{
  Load("worlds/shapes.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr sphereModel = world->GetModel("sphere");
  physics::ModelPtr boxModel = world->GetModel("box");

  EXPECT_TRUE(sphereModel != NULL);
  EXPECT_TRUE(boxModel != NULL);

  world->RemoveModel(sphereModel);
  world->RemoveModel("box");

  sphereModel = world->GetModel("sphere");
  boxModel = world->GetModel("box");

  EXPECT_FALSE(sphereModel != NULL);
  EXPECT_FALSE(boxModel != NULL);
}

/////////////////////////////////////////////////
TEST_F(WorldTest, RemoveModelUnPaused)
{
  Load("worlds/shapes.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr sphereModel = world->GetModel("sphere");
  physics::ModelPtr boxModel = world->GetModel("box");

  EXPECT_TRUE(sphereModel != NULL);
  EXPECT_TRUE(boxModel != NULL);

  world->Step(1);
  world->RemoveModel(sphereModel);
  world->RemoveModel("box");

  sphereModel = world->GetModel("sphere");
  boxModel = world->GetModel("box");

  EXPECT_FALSE(sphereModel != NULL);
  EXPECT_FALSE(boxModel != NULL);
}

/////////////////////////////////////////////////
/// \brief Check if WorldUpdateBegin, BeforePhysicsUpdate and WorldUpdateEnd
/// events are called, and if the BeforePhysicsUpdate event is really called
/// before the physics engine update happens
TEST_F(WorldTest, CheckWorldEventsWork)
{
  Load("worlds/shapes.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr sphereModel = world->GetModel("sphere");
  ASSERT_TRUE(sphereModel != NULL);

  physics::LinkPtr link = sphereModel->GetLink("link");
  ASSERT_TRUE(link != NULL);

  // run the world for a while just to stabilize
  world->Step(10);

  // initial pose of the link
  ignition::math::Pose3d initialPose = link->GetWorldPose().Ign();

  // connect to the world events
  event::ConnectionPtr worldUpdateBeginEventConnection =
    event::Events::ConnectWorldUpdateBegin(&onWorldUpdateBegin);

  event::ConnectionPtr beforePhysicsUpdateConnection =
    event::Events::ConnectBeforePhysicsUpdate(&beforePhysicsUpdate);

  event::ConnectionPtr worldUpdateEndEventConnection =
    event::Events::ConnectWorldUpdateEnd(&onWorldUpdateEnd);

  // iterate for a while pushing to the ball and check the events get called
  // and that the pose changes only after BeforePhysicUpdate is called.
  for (size_t i = 0; i < 100; ++i)
  {
    ASSERT_FALSE(g_updateBeginCalled);
    ASSERT_FALSE(g_beforePhysicsUpdateCalled);
    ASSERT_FALSE(g_updateEndCalled);

    // push to the ball
    link->AddForce(ignition::math::Vector3d(1000, 0, 0));

    world->Step(1);

    // pose after the physics update
    ignition::math::Pose3d poseAfterUpdate = link->GetWorldPose().Ign();

    // initial pose and pose before physics update should be the same
    EXPECT_EQ(initialPose.Pos(), g_poseBeforeUpdate.Pos());

    // pose before physics update and after it should be different
    EXPECT_GT((g_poseAfterUpdate - g_poseBeforeUpdate).Pos().Abs().Sum(), 1e-9);

    // the events should get called
    EXPECT_TRUE(g_updateBeginCalled);
    EXPECT_TRUE(g_beforePhysicsUpdateCalled);
    EXPECT_TRUE(g_updateEndCalled);

    g_updateBeginCalled = false;
    g_beforePhysicsUpdateCalled = false;
    g_updateEndCalled = false;

    // remember the current pose to compare it in the next iteration
    initialPose = poseAfterUpdate;
  }

  // disconnect from world events
  worldUpdateBeginEventConnection.reset();
  beforePhysicsUpdateConnection.reset();
  worldUpdateEndEventConnection.reset();
}

/////////////////////////////////////////////////
TEST_F(WorldTest, URI)
{
  Load("worlds/nested_model.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  EXPECT_EQ(world->URI().Str(), "data://world/default");

  // Check a light.
  auto light = world->Light("sun");
  ASSERT_TRUE(light != NULL);
  EXPECT_EQ(light->URI().Str(), "data://world/default/light/sun");

  // Check a model.
  auto model = world->GetModel("ground_plane");
  ASSERT_TRUE(model != NULL);
  EXPECT_EQ(model->URI().Str(), "data://world/default/model/ground_plane");

  auto link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);
  EXPECT_EQ(link->URI().Str(),
      "data://world/default/model/ground_plane/link/link");

  // Check a nested model.
  model = world->GetModel("model_00");
  ASSERT_TRUE(model != NULL);
  EXPECT_EQ(model->URI().Str(), "data://world/default/model/model_00");

  link = model->GetLink("link_00");
  ASSERT_TRUE(link != NULL);
  EXPECT_EQ(link->URI().Str(),
      "data://world/default/model/model_00/link/link_00");

  auto nestedModel = model->NestedModel("model_01");
  ASSERT_TRUE(nestedModel != NULL);
  EXPECT_EQ(nestedModel->URI().Str(),
      "data://world/default/model/model_00/model/model_01");

  link = nestedModel->GetLink("link_01");
  ASSERT_TRUE(link != NULL);
  EXPECT_EQ(link->URI().Str(),
      "data://world/default/model/model_00/model/model_01/link/link_01");
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, WorldTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
