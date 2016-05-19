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
#include <string.h>
#include "gazebo/math/Helpers.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/test/ServerFixture.hh"
#include "images_cmp.h"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class FactoryTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void BoxSdf(const std::string &_physicsEngine);
  public: void Box(const std::string &_physicsEngine);
  public: void Sphere(const std::string &_physicsEngine);
  public: void Cylinder(const std::string &_physicsEngine);
  public: void Clone(const std::string &_physicsEngine);
};

///////////////////////////////////////////////////
// Verify that sdf is retained by entities spawned
// via factory messages. A change between 1.6, 1.7
// caused entities to lose their sdf data
// (see issue #651)
void FactoryTest::BoxSdf(const std::string &_physicsEngine)
{
  ignition::math::Pose3d setPose;
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  unsigned int entityCount = 6;

  for (unsigned int i = 0; i < entityCount; ++i)
  {
    std::ostringstream name;
    name << "test_box_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond(0, 0, 0));
    SpawnBox(name.str(), ignition::math::Vector3d(1, 1, 1), setPose.Pos(),
        setPose.Rot().Euler());
  }

  // This loop must be separate from the previous loop to cause
  // the failure.
  for (unsigned int i = 0; i < entityCount; ++i)
  {
    std::ostringstream name;
    name << "test_box_" << i;

    physics::ModelPtr model = world->ModelByName(name.str());
    ASSERT_TRUE(model != NULL);
    msgs::Model msg;
    model->FillMsg(msg);
    EXPECT_TRUE(msg.has_pose());
    // gzerr << msg.DebugString() << '\n';
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, BoxSdf)
{
  BoxSdf(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::Box(const std::string &_physicsEngine)
{
  ignition::math::Pose3d setPose, testPose;
  Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_box_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond(0, 0, 0));
    SpawnBox(name.str(), ignition::math::Vector3d(1, 1, 1), setPose.Pos(),
        setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(ignition::math::equal(
          testPose.Pos().X(), setPose.Pos().X(), 0.1));
    EXPECT_TRUE(ignition::math::equal(
          testPose.Pos().Y(), setPose.Pos().Y(), 0.1));
    EXPECT_TRUE(ignition::math::equal(
          testPose.Pos().Z(), setPose.Pos().Z(), 0.1));
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, Box)
{
  Box(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::Sphere(const std::string &_physicsEngine)
{
  ignition::math::Pose3d setPose, testPose;
  Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_sphere_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond(0, 0, 0));
    SpawnSphere(name.str(), setPose.Pos(), setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().X(),
          setPose.Pos().X(), 0.1));
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().Y(),
          setPose.Pos().Y(), 0.1));
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().Z(),
          setPose.Pos().Z(), 0.1));
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, Sphere)
{
  Sphere(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::Cylinder(const std::string &_physicsEngine)
{
  ignition::math::Pose3d setPose, testPose;
  Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_cylinder_" << i;
    setPose.Set(
        ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond(0, 0, 0));
    SpawnCylinder(name.str(), setPose.Pos(), setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().X(),
          setPose.Pos().X(), 0.1));
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().Y(),
          setPose.Pos().Y(), 0.1));
    EXPECT_TRUE(ignition::math::equal(testPose.Pos().Z(),
          setPose.Pos().Z(), 0.1));
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, Cylinder)
{
  Cylinder(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::Clone(const std::string &_physicsEngine)
{
  ignition::math::Pose3d testPose;
  Load("worlds/pr2.world", true, _physicsEngine);

  // clone the pr2
  std::string name = "pr2";
  msgs::Factory msg;
  ignition::math::Pose3d clonePose;
  clonePose.Set(ignition::math::Vector3d(2, 3, 0.5),
      ignition::math::Quaterniond(0, 0, 0));
  msgs::Set(msg.mutable_pose(), clonePose);
  msg.set_clone_model_name(name);
  this->factoryPub->Publish(msg);

  // Wait for the pr2 clone to spawn
  std::string cloneName = name + "_clone";
  this->WaitUntilEntitySpawn(cloneName, 100, 100);

  EXPECT_TRUE(this->HasEntity(cloneName));
  testPose = EntityPose(cloneName);
  EXPECT_TRUE(ignition::math::equal(testPose.Pos().X(),
        clonePose.Pos().X(), 0.1));
  EXPECT_TRUE(ignition::math::equal(testPose.Pos().Y(),
        clonePose.Pos().Y(), 0.1));
  EXPECT_TRUE(ignition::math::equal(testPose.Pos().Z(),
        clonePose.Pos().Z(), 0.1));

  // Verify properties of the pr2 clone with the original model.
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check model
  physics::ModelPtr model = world->ModelByName(name);
  ASSERT_TRUE(model != NULL);
  physics::ModelPtr modelClone = world->ModelByName(cloneName);
  ASSERT_TRUE(modelClone != NULL);
  EXPECT_EQ(model->JointCount(), modelClone->JointCount());
  EXPECT_EQ(model->Links().size(), modelClone->Links().size());
  EXPECT_EQ(model->SensorCount(), modelClone->SensorCount());
  EXPECT_EQ(model->PluginCount(), modelClone->PluginCount());
  EXPECT_EQ(model->AutoDisable(), modelClone->AutoDisable());

  // Check links
  physics::Link_V links = model->Links();
  physics::Link_V linkClones = modelClone->Links();
  for (unsigned int i = 0; i < links.size(); ++i)
  {
    physics::LinkPtr link = links[i];
    physics::LinkPtr linkClone = linkClones[i];

    EXPECT_EQ(link->SensorCount(), linkClone->SensorCount());
    EXPECT_EQ(link->Kinematic(), linkClone->Kinematic());

    // Check collisions
    physics::Collision_V collisions = link->Collisions();
    physics::Collision_V collisionClones = linkClone->Collisions();
    EXPECT_EQ(collisions.size(), collisionClones.size());
    for (unsigned int j = 0; j < collisions.size(); ++j)
    {
      physics::CollisionPtr collision = collisions[j];
      physics::CollisionPtr collisionClone = collisionClones[j];
      EXPECT_EQ(collision->ShapeType(), collisionClone->ShapeType());
      EXPECT_EQ(collision->MaxContacts(), collisionClone->MaxContacts());

      // Check surface
      physics::SurfaceParamsPtr surface = collision->Surface();
      physics::SurfaceParamsPtr cloneSurface = collisionClone->Surface();
      EXPECT_EQ(surface->collideWithoutContact,
          cloneSurface->collideWithoutContact);
      EXPECT_EQ(surface->collideWithoutContactBitmask,
          cloneSurface->collideWithoutContactBitmask);
    }

    // Check inertial
    physics::Inertial inertial = link->Inertia();
    physics::Inertial inertialClone = linkClone->Inertia();
    EXPECT_EQ(inertial.Mass(), inertialClone.Mass());
    EXPECT_EQ(inertial.CoG(), inertialClone.CoG());
    EXPECT_EQ(inertial.PrincipalMoments(),
        inertialClone.PrincipalMoments());
    EXPECT_EQ(inertial.ProductsOfInertia(),
        inertialClone.ProductsOfInertia());
  }

  // Check joints
  physics::Joint_V joints = model->Joints();
  physics::Joint_V jointClones = modelClone->Joints();
  for (unsigned int i = 0; i < joints.size(); ++i)
  {
    physics::JointPtr joint = joints[i];
    physics::JointPtr jointClone = jointClones[i];
    EXPECT_EQ(joint->AngleCount(), jointClone->AngleCount());
    for (unsigned j = 0; j < joint->AngleCount(); ++j)
    {
      EXPECT_EQ(joint->UpperLimit(j), jointClone->UpperLimit(j));
      EXPECT_EQ(joint->LowerLimit(j), jointClone->LowerLimit(j));
      EXPECT_EQ(joint->EffortLimit(j), jointClone->EffortLimit(j));
      EXPECT_EQ(joint->VelocityLimit(j), jointClone->VelocityLimit(j));
      EXPECT_EQ(joint->StopStiffness(j), jointClone->StopStiffness(j));
      EXPECT_EQ(joint->StopDissipation(j),
          jointClone->StopDissipation(j));
      EXPECT_EQ(joint->LocalAxis(j), jointClone->LocalAxis(j));
      EXPECT_EQ(joint->Damping(j), jointClone->Damping(j));
    }
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, Clone)
{
  Clone(GetParam());
}

// Disabling this test for now. Different machines return different
// camera images. Need a better way to evaluate rendered content.
// TEST_F(FactoryTest, Camera)
// {
/*
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
    return;

  ignition::math::Pose3d setPose, testPose;
  Load("worlds/empty.world");
  setPose.Set(ignition::math::Vector3d(-5, 0, 5), ignition::math::Quaterniond(0, GZ_DTOR(15), 0));
  SpawnCamera("camera_model", "camera_sensor2", setPose.pos,
      setPose.Rot().Euler());

  unsigned char *img = NULL;
  unsigned int width;
  unsigned int height;
  GetFrame("camera_sensor2", &img, width, height);
  ASSERT_EQ(width, static_cast<unsigned int>(320));
  ASSERT_EQ(height, static_cast<unsigned int>(240));

  unsigned int diffMax = 0;
  unsigned int diffSum = 0;
  double diffAvg = 0;
  ImageCompare(&img, &empty_world_camera1,
      width, height, 3, diffMax, diffSum, diffAvg);
  // PrintImage("empty_world_camera1", &img, width, height, 3);
  ASSERT_LT(diffSum, static_cast<unsigned int>(100));
  ASSERT_EQ(static_cast<unsigned int>(0), diffMax);
  ASSERT_EQ(0.0, diffAvg);
  */
// }

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FactoryTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
