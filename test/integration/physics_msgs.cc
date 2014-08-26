/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsMsgsTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void MoveTool(const std::string &_physicsEngine);
  public: void SetGravity(const std::string &_physicsEngine);
  public: void LinkProperties(const std::string &_physicsEngine);
  public: void LinkPose(const std::string &_physicsEngine);
  public: void SimpleShapeResize(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void PhysicsMsgsTest::SetGravity(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // Set Gravity by publishing to "~/physics"
  transport::PublisherPtr physicsPub =
    this->node->Advertise<msgs::Physics>("~/physics");
  msgs::Physics msg;
  // it doesn't actually seem to matter what type you set
  msg.set_type(msgs::Physics::Type_MIN);

  std::vector<math::Vector3> gravity;
  gravity.push_back(math::Vector3(0, 0, 9.81));
  gravity.push_back(math::Vector3(0, 0, -20));
  gravity.push_back(math::Vector3(0, 0, 20));
  gravity.push_back(math::Vector3(0, 0, 0));
  gravity.push_back(math::Vector3(0, 0, -9.81));
  gravity.push_back(math::Vector3(1, 1, 9.81));
  gravity.push_back(math::Vector3(2, 3, -20));
  gravity.push_back(math::Vector3(2, -3, 20));
  gravity.push_back(math::Vector3(-2, 3, 0));
  gravity.push_back(math::Vector3(-2, -3, -9.81));

  for (std::vector<math::Vector3>::iterator iter = gravity.begin();
       iter != gravity.end(); ++iter)
  {
    msgs::Set(msg.mutable_gravity(), *iter);
    physicsPub->Publish(msg);

    while (*iter != physics->GetGravity())
    {
      world->Step(1);
      common::Time::MSleep(1);
    }

    EXPECT_EQ(*iter, physics->GetGravity());
  }
}

/////////////////////////////////////////////////
void PhysicsMsgsTest::MoveTool(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // set gravity to zero
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  physics->SetGravity(math::Vector3::Zero);

  // spawn a box
  std::string name = "test_box";
  double z0 = 5;
  math::Vector3 pos = math::Vector3(0, 0, z0);
  math::Vector3 size = math::Vector3(1, 1, 1);
  SpawnBox(name, size, pos, math::Vector3::Zero);
  gzdbg << "SpawnBox called" << std::endl;

  // advertise on "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  // list of poses to move to
  std::vector<math::Pose> poses;
  poses.push_back(math::Pose(5, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 8, z0, 0, 0, 0));
  poses.push_back(math::Pose(-99, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 999, z0, 0, 0, 0));
  poses.push_back(math::Pose(123.456, 456.123, z0*10, 0.1, -0.2, 0.3));
  poses.push_back(math::Pose(-123.456, 456.123, z0*10, 0.2, 0.4, -0.6));
  poses.push_back(math::Pose(123.456, -456.123, z0*10, 0.3, -0.6, 0.9));
  poses.push_back(math::Pose(-123.456, -456.123, z0*10, -0.4, 0.8, -1.2));

  physics::ModelPtr model = world->GetModel(name);
  ASSERT_TRUE(model != NULL);

  {
    math::Pose initialPose = model->GetWorldPose();
    EXPECT_EQ(pos, initialPose.pos);
  }

  {
    msgs::Model msg;
    msg.set_name(name);
    msg.set_id(model->GetId());

    for (std::vector<math::Pose>::iterator iter = poses.begin();
         iter != poses.end(); ++iter)
    {
      msgs::Set(msg.mutable_pose(), *iter);
      modelPub->Publish(msg);

      while (*iter != model->GetWorldPose())
      {
        world->Step(1);
        common::Time::MSleep(1);
      }

      // Take a few steps to verify the correct model pose.
      // dart has a failure mode that was not exposed without
      // this change to the test.
      world->Step(10);

      EXPECT_EQ(*iter, model->GetWorldPose());
    }
  }
}

/////////////////////////////////////////////////
void PhysicsMsgsTest::LinkProperties(const std::string &_physicsEngine)
{
  // TODO simbody currently fails this test
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting LinkProperties test for Simbody" << std::endl;
    return;
  }

  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // spawn a box
  std::string name = "test_box";
  double z0 = 5;
  math::Vector3 pos = math::Vector3(0, 0, z0);
  math::Vector3 size = math::Vector3(1, 1, 1);
  SpawnBox(name, size, pos, math::Vector3::Zero);

  // advertise on "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  physics::ModelPtr model = world->GetModel(name);
  ASSERT_TRUE(model != NULL);

  // change gravity mode and verify the msg gets through
  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link != NULL);

  // check default link gravity and kinematic properties
  {
    // gravity mode should be enabled by default
    EXPECT_TRUE(link->GetGravityMode());

    // TODO Bullet currently fails this test
    if (_physicsEngine != "bullet")
    {
      // kinematic mode should be disabled by default
      EXPECT_TRUE(!link->GetKinematic());
    }
    else
    {
      gzerr << "Skipping LinkProperties's kinematic test for Bullet" <<
          std::endl;
    }

    // self collide mode should be disabled by default
    EXPECT_TRUE(!link->GetSelfCollide());
  }

  {
    // change gravity mode and verify the msg gets through
    msgs::Model msg;
    msg.set_name(name);
    msg.set_id(model->GetId());

    msgs::Link *linkMsg = msg.add_link();
    linkMsg->set_id(link->GetId());
    linkMsg->set_name(link->GetScopedName());

    bool newGravityMode = false;
    EXPECT_TRUE(newGravityMode != link->GetGravityMode());
    linkMsg->set_gravity(newGravityMode);
    modelPub->Publish(msg);

    int sleep = 0;
    int maxSleep = 50;
    while (link->GetGravityMode() != newGravityMode && sleep < maxSleep)
    {
      world->Step(1);
      common::Time::MSleep(100);
      sleep++;
    }
    ASSERT_TRUE(link->GetGravityMode() == newGravityMode);
  }

  // TODO Bullet and DART currently fail this test
  if (_physicsEngine != "bullet" && _physicsEngine != "dart")
  {
    // change kinematic mode and verify the msg gets through
    msgs::Model msg;
    msg.set_name(name);
    msg.set_id(model->GetId());

    msgs::Link *linkMsg = msg.add_link();
    linkMsg->set_id(link->GetId());
    linkMsg->set_name(link->GetScopedName());

    bool newKinematicMode = true;
    EXPECT_TRUE(newKinematicMode != link->GetKinematic());
    linkMsg->set_kinematic(newKinematicMode);
    modelPub->Publish(msg);

    int sleep = 0;
    int maxSleep = 50;
    while (link->GetKinematic() != newKinematicMode && sleep < maxSleep)
    {
      world->Step(1);
      common::Time::MSleep(100);
      sleep++;
    }
    EXPECT_TRUE(link->GetKinematic() == newKinematicMode);
  }
  else
  {
    gzerr << "Skipping LinkProperties's kinematic test for "
          << _physicsEngine << std::endl;
  }

  {
    // change self collide mode and verify the msg gets through
    msgs::Model msg;
    msg.set_name(name);
    msg.set_id(model->GetId());

    msgs::Link *linkMsg = msg.add_link();
    linkMsg->set_id(link->GetId());
    linkMsg->set_name(link->GetName());

    bool newSelfCollideMode = true;
    EXPECT_TRUE(newSelfCollideMode != link->GetSelfCollide());
    linkMsg->set_self_collide(newSelfCollideMode);
    modelPub->Publish(msg);

    int sleep = 0;
    int maxSleep = 50;
    while (link->GetSelfCollide() != newSelfCollideMode && sleep < maxSleep)
    {
      world->Step(1);
      common::Time::MSleep(100);
      sleep++;
    }
    EXPECT_TRUE(link->GetSelfCollide() == newSelfCollideMode);
  }
}

/////////////////////////////////////////////////
void PhysicsMsgsTest::LinkPose(const std::string &_physicsEngine)
{
  Load("worlds/multilink_shape.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // set gravity to zero
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  physics->SetGravity(math::Vector3::Zero);

  // advertise on "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  double z0 = 5;
  // list of poses to move to
  std::vector<math::Pose> poses;
  poses.push_back(math::Pose(5, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 8, z0, 0, 0, 0));
  poses.push_back(math::Pose(-99, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 999, z0, 0, 0, 0));
  poses.push_back(math::Pose(123.456, 456.123, z0*10, 0.1, -0.2, 0.3));
  poses.push_back(math::Pose(-123.456, 456.123, z0*10, 0.2, 0.4, -0.6));
  poses.push_back(math::Pose(123.456, -456.123, z0*10, 0.3, -0.6, 0.9));
  poses.push_back(math::Pose(-123.456, -456.123, z0*10, -0.4, 0.8, -1.2));

  std::string name = "multilink";
  physics::ModelPtr model = world->GetModel(name);
  ASSERT_TRUE(model != NULL);

  {
    for (unsigned int i = 0; i < model->GetLinks().size(); ++i)
    {
      physics::LinkPtr link = model->GetLinks()[i];
      ASSERT_TRUE(link != NULL);

      if (link->IsCanonicalLink())
        continue;

      for (std::vector<math::Pose>::iterator iter = poses.begin();
           iter != poses.end(); ++iter)
      {
        msgs::Model msg;
        msg.set_name(name);
        msg.set_id(model->GetId());

        msgs::Link *linkMsg = msg.add_link();
        linkMsg->set_id(link->GetId());
        linkMsg->set_name(link->GetScopedName());

        msgs::Set(linkMsg->mutable_pose(), *iter);
        modelPub->Publish(msg);

        int sleep = 0;
        int maxSleep = 50;
        while (*iter != link->GetRelativePose() && sleep < maxSleep)
        {
          world->Step(1);
          common::Time::MSleep(1);
        }

        // Take a few steps to verify the correct link pose.
        // dart has a failure mode that was not exposed without
        // this change to the test.
        world->Step(10);

        EXPECT_EQ(*iter, link->GetRelativePose());
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////
// SimpleShapeResize: (Test adapted from PhysicsTest::SpawnDrop)
// Load a world, check that gravity points along z axis, spawn simple
// shapes (box, sphere, cylinder), resize them to be smaller, verify that they
// then start falling.
////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::SimpleShapeResize(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Aborting test for DART, see issue #1175.\n";
    return;
  }

  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // get physics time step
  double dt = physics->GetMaxStepSize();
  EXPECT_GT(dt, 0);

  // spawn some simple shapes with unit size
  double z0 = 0.5;
  std::map<std::string, math::Vector3> modelPos;
  modelPos["test_box"] = math::Vector3(0, 0, z0);
  modelPos["test_sphere"] = math::Vector3(4, 0, z0);
  modelPos["test_cylinder"] = math::Vector3(8, 0, z0);

  SpawnBox("test_box", math::Vector3(1, 1, 1), modelPos["test_box"],
      math::Vector3::Zero);
  SpawnSphere("test_sphere", modelPos["test_sphere"], math::Vector3::Zero);
  SpawnCylinder("test_cylinder", modelPos["test_cylinder"],
      math::Vector3::Zero);

  // spawn another set of shapes and use messages to resize these
  modelPos["test_box2"] = math::Vector3(0, 9, z0);
  modelPos["test_sphere2"] = math::Vector3(4, 9, z0);
  modelPos["test_cylinder2"] = math::Vector3(8, 9, z0);

  SpawnBox("test_box2", math::Vector3(1, 1, 1), modelPos["test_box2"],
      math::Vector3::Zero);
  SpawnSphere("test_sphere2", modelPos["test_sphere2"], math::Vector3::Zero);
  SpawnCylinder("test_cylinder2", modelPos["test_cylinder2"],
      math::Vector3::Zero);

  // advertise on "~/model/modify" to generate resize messages
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  int steps = 2;
  physics::ModelPtr model;
  math::Pose pose1, pose2;
  math::Vector3 vel1, vel2;
  double x0, y0;

  // Allow objects to settle on ground_plane
  world->Step(100);

  // Verify the initial model pose is where we set it to be.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->GetModel(name);
    EXPECT_TRUE(model != NULL);

    pose1 = model->GetWorldPose();
    x0 = modelPos[name].x;
    y0 = modelPos[name].y;

    EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
    EXPECT_NEAR(pose1.pos.y, y0, PHYSICS_TOL);
    EXPECT_NEAR(pose1.pos.z, z0, PHYSICS_TOL);
  }

  // resize model to half of it's size
  double scaleFactor = 0.5;
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    model = world->GetModel(name);
    if (*(name.rbegin()) == '2')
    {
      // Use a message to resize this one
      msgs::Model msg;
      msg.set_name(name);
      msg.set_id(model->GetId());
      msgs::Set(msg.mutable_scale(), scaleFactor * math::Vector3::One);
      modelPub->Publish(msg);
    }
    else
    {
      // Use physics API to resize
      model->SetScale(scaleFactor * math::Vector3::One);
    }
  }

  // Predict time of contact with ground plane.
  double tHit = sqrt(2*(z0-0.5*scaleFactor) / (-g.z));
  // Time to advance, allow 0.5 s settling time.
  // This assumes inelastic collisions with the ground.
  double dtHit = tHit+0.5 - world->GetSimTime().Double();
  steps = ceil(dtHit / dt);
  EXPECT_GT(steps, 0);
  world->Step(steps);

  // Issue #856, simbody doesn't support shape resizes.
  if (_physicsEngine == "simbody")
  {
    gzerr << "Aborting test since simbody doesn't support shape resizes (#856)"
          << std::endl;
    return;
  }

  // This loop checks the velocity and pose of each model 0.5 seconds
  // after the time of predicted ground contact. The pose is expected to be
  // underneath the initial pose.
  for (std::map<std::string, math::Vector3>::iterator iter = modelPos.begin();
    iter != modelPos.end(); ++iter)
  {
    std::string name = iter->first;
    // Make sure the model is loaded
    model = world->GetModel(name);
    if (model != NULL)
    {
      gzdbg << "Check ground contact of model " << name << '\n';
      // Check that model is resting on ground
      pose1 = model->GetWorldPose();
      x0 = modelPos[name].x;
      y0 = modelPos[name].y;
      EXPECT_NEAR(pose1.pos.x, x0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.y, y0, PHYSICS_TOL);
      EXPECT_NEAR(pose1.pos.z, 0.5*scaleFactor, PHYSICS_TOL);
    }
    else
    {
      gzerr << "Error loading model " << name << '\n';
      EXPECT_TRUE(model != NULL);
    }
  }
}


/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, SetGravity)
{
  SetGravity(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, MoveTool)
{
  MoveTool(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, LinkProperties)
{
  LinkProperties(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, LinkPose)
{
  LinkPose(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, SimpleShapeResize)
{
  SimpleShapeResize(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
