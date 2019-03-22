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
#include <map>
#include <string>
#include <ignition/math/Helpers.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

const double g_big = 1e17;
const double g_physics_tol = 1e-2;

class PhysicsCollisionTest : public ServerFixture,
                             public testing::WithParamInterface<const char*>
{
  /// \brief Test Collision::GetBoundingBox.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetBoundingBox(const std::string &_physicsEngine);

  /// \brief Spawn identical models with different collision pose offsets
  /// and verify that they have matching behavior.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void PoseOffsets(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsCollisionTest::GetBoundingBox(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Bounding boxes not yet working with "
          << _physicsEngine
          << ", see issue #1148"
          << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check bounding box of ground plane.
  // Skip this test for DART because planes aren't handled properly yet.
  // For this test to pass for DART, the following values would need to be set:
  //    big = 1049;
  //    z = 1050;
  if (_physicsEngine != "dart")
  {
    double big = g_big;
    double z = 0.0;

    physics::ModelPtr model = world->ModelByName("ground_plane");
    ASSERT_TRUE(model != nullptr);
    ignition::math::Box box = model->BoundingBox();
    EXPECT_LT(box.Min().X(), -big);
    EXPECT_LT(box.Min().Y(), -big);
    EXPECT_LT(box.Min().Z(), -big);
    EXPECT_GT(box.Max().X(), big);
    EXPECT_GT(box.Max().Y(), big);
    EXPECT_DOUBLE_EQ(box.Max().Z(), z);
  }

  // Insert model which forms a t-shape, made up of two long boxes.
  // The second box will have to be translated relative to the first to
  // form the "roof" of the T. This tests whether the relative transforms
  // of the collision shapes are considered when computing the bounding box.
  // If they are not, the result would be that of a cross-shape instead.

  std::ostringstream sdfStream;
  sdfStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='model_tshape'>"
    << "<static> true </static>"
    << "<pose>0 0 0 0 0 0</pose>"
    << "<link name=\"link\">"
      << "<collision name=\"pose\">"
        << "<pose>0 0.5 0 0 0 0</pose>"
        << "<geometry>"
          << "<box>"
            << "<size>0.1 1 0.1</size>"
          << "</box>"
        << "</geometry>"
      << "</collision>"
      << "<collision name=\"top\">"
        << "<pose>0 0.95 0 0 0 0</pose>"
        << "<geometry>"
          << "<box>"
            << "<size>1 0.1 0.1</size>"
          << "</box>"
        << "</geometry>"
      << "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(sdfStream.str());

  {
    physics::ModelPtr model = world->ModelByName("model_tshape");
    ASSERT_TRUE(model != nullptr);
    ignition::math::Box box = model->BoundingBox();
    gzdbg << "Bounding box for " << _physicsEngine << ": " << box << std::endl;
    static double tol = 1e-03;
    EXPECT_NEAR(box.Min().X(), -0.5, tol);
    EXPECT_NEAR(box.Min().Y(), 0, tol);
    EXPECT_NEAR(box.Min().Z(), -0.05, tol);
    EXPECT_NEAR(box.Max().X(), 0.5, tol);
    EXPECT_NEAR(box.Max().Y(), 1, tol);
    EXPECT_NEAR(box.Max().Z(), 0.05, tol);
  }
}

/////////////////////////////////////////////////
void PhysicsCollisionTest::PoseOffsets(const std::string &_physicsEngine)
{
  Load("worlds/collision_pose_offset.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Box size
  const double dy = 0.4;
  const double dz = 0.9;

  int modelCount = 0;
  auto models = world->Models();
  for (auto model : models)
  {
    ASSERT_TRUE(model != nullptr);
    auto name = model->GetName();
    if (0 != name.compare(0, 4, "box_"))
    {
      continue;
    }
    modelCount++;

    int i = std::stoi(name.substr(4, 5));

    auto link = model->GetLink();
    ASSERT_TRUE(link != nullptr);

    // The following line used to be
    // const unsigned int index = 0;
    // but for some reason that version caused a C2666 error
    // (overloading ambiguity) in Visual Studio 2015 in the following call:
    // auto collision = link->GetCollision(index);
    unsigned int index = 0;
    auto collision = link->GetCollision(index);
    ASSERT_TRUE(collision != nullptr);

    // for box_3, also get link_2 and its collision
    auto link2 = model->GetLink("link_2");
    physics::CollisionPtr collision2;
    if (i == 3)
    {
      EXPECT_NE(link2, nullptr);
      collision2 = link2->GetCollision(index);
      EXPECT_NE(collision2, nullptr);
    }
    else
    {
      EXPECT_EQ(link2, nullptr);
    }

    // i=0: rotated model pose
    //  expect collision pose to match model pose
    if (i == 0)
    {
      EXPECT_EQ(model->WorldPose(), collision->WorldPose());
    }
    // i=1: rotated link pose
    //  expect collision pose to match link pose
    else if (i == 1)
    {
      EXPECT_EQ(link->WorldPose(), collision->WorldPose());
    }
    // i=2: rotated collision pose
    //  expect collision position to match link position
    else if (i == 2)
    {
      EXPECT_EQ(link->WorldPose().Pos(), collision->WorldPose().Pos());
    }
    // i=3: offset collision pose
    //  expect collision postion to match link position plus offset
    else if (i == 3)
    {
      ignition::math::Pose3d collisionPose(0, 0, dz, 0, 0, 0);
      EXPECT_EQ(link->WorldPose().Pos() + collisionPose.Pos(),
                collision->WorldPose().Pos());

      // collision2 pose should match link2 pose
      EXPECT_EQ(link2->WorldPose().Pos(),
                collision2->WorldPose().Pos());
    }
  }
  EXPECT_EQ(modelCount, 4);

  const double t0 = 1.5;
  const double dt = 1e-3;
  const int steps = floor(t0 / dt);
  world->Step(steps);

  for (auto model : models)
  {
    ASSERT_TRUE(model != nullptr);
    auto name = model->GetName();
    if (0 != name.compare(0, 4, "box_"))
    {
      continue;
    }

    int i = std::stoi(name.substr(4, 5));

    auto link = model->GetLink();
    ASSERT_TRUE(link != nullptr);

    unsigned int index = 0;
    auto collision = link->GetCollision(index);
    ASSERT_TRUE(collision != nullptr);

    // for box_3, also get link_2 and its collision
    auto link2 = model->GetLink("link_2");
    physics::CollisionPtr collision2;
    if (i == 3)
    {
      EXPECT_NE(link2, nullptr);
      collision2 = link2->GetCollision(index);
      EXPECT_NE(collision2, nullptr);
    }
    else
    {
      EXPECT_EQ(link2, nullptr);
    }

    // For 0-2, drop and expect box to rest at specific height
    if (i <= 2)
    {
      EXPECT_NEAR(collision->WorldPose().Pos().Z(), dy/2, g_physics_tol);
    }
    else
    {
      EXPECT_NEAR(collision->WorldPose().Pos().Z(), dz/2, g_physics_tol);
      EXPECT_NEAR(collision2->WorldPose().Pos().Z(), dz/2, g_physics_tol);
    }
  }
}

/////////////////////////////////////////////////
TEST_F(PhysicsCollisionTest, ModelSelfCollide)
{
  // self_collide is only implemented in ODE
  Load("worlds/model_self_collide.world", true, "ode");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);

  auto g = world->Gravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.X(), 0);
  EXPECT_EQ(g.Y(), 0);
  EXPECT_LE(g.Z(), -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // 4 models: all_collide, some_collide, no_collide, and explicit_no_collide
  std::map<std::string, physics::ModelPtr> models;
  models["all_collide"]         = physics::ModelPtr();
  models["some_collide"]        = physics::ModelPtr();
  models["no_collide"]          = physics::ModelPtr();
  models["explicit_no_collide"] = physics::ModelPtr();
  for (auto &iter : models)
  {
    gzdbg << "Getting model " << iter.first << std::endl;
    iter.second = world->ModelByName(iter.first);
    ASSERT_TRUE(iter.second != NULL);
  }

  // Step forward 0.2 s
  double stepTime = 0.2;
  unsigned int steps = floor(stepTime / dt);
  world->Step(steps);

  // Expect boxes to be falling
  double fallVelocity = g.Z() * stepTime;
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check falling: " << link->GetScopedName() << std::endl;
      EXPECT_LT(link->WorldLinearVel().Z(), fallVelocity*(1-g_physics_tol));
    }
  }

  // Another 3000 steps should put the boxes at rest
  world->Step(3000);

  // Expect 3 boxes to be stationary
  for (auto &iter : models)
  {
    auto links = iter.second->GetLinks();
    for (auto &link : links)
    {
      ASSERT_TRUE(link != NULL);
      gzdbg << "Check resting: " << link->GetScopedName() << std::endl;
      EXPECT_NEAR(link->WorldLinearVel().Z(), 0, g_physics_tol);
    }
  }

  gzdbg << "Check resting positions" << std::endl;

  // link2 of all_collide should have the highest z-coordinate (around 3)
  EXPECT_NEAR(models["all_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              2.5, g_physics_tol);

  // link2 of some_collide should have a middling z-coordinate (around 2)
  EXPECT_NEAR(models["some_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              1.5, g_physics_tol);

  // link2 of no_collide should have a low z-coordinate (around 1)
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
              0.5, g_physics_tol);

  // link2 of explicit_no_collide should have the same z-coordinate as above
  EXPECT_NEAR(models["no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
     models["explicit_no_collide"]->GetLink("link2")->WorldPose().Pos().Z(),
     g_physics_tol);

  Unload();
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, GetBoundingBox)
{
  GetBoundingBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsCollisionTest, PoseOffsets)
{
  PoseOffsets(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsCollisionTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
