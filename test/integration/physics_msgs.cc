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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

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
  public: void LinkVisualMsg(const std::string &_physicsEngine);
  public: void JointMsg(const std::string &_physicsEngine);
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
    msgs::Set(msg.mutable_gravity(), (*iter).Ign());
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
      msgs::Set(msg.mutable_pose(), (*iter).Ign());
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

        msgs::Set(linkMsg->mutable_pose(), (*iter).Ign());
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
  modelPos["test_non_unit_box"] = math::Vector3(16, 0, z0);

  std::map<std::string, math::Vector3> modelSize;
  modelSize["test_box"] = math::Vector3::One;
  modelSize["test_sphere"] = math::Vector3::One;
  modelSize["test_cylinder"] = math::Vector3::One;
  modelSize["test_non_unit_box"] = math::Vector3(3, 8, 1.0);

  SpawnBox("test_box", math::Vector3(1, 1, 1), modelPos["test_box"],
      math::Vector3::Zero);
  SpawnSphere("test_sphere", modelPos["test_sphere"], math::Vector3::Zero);
  SpawnCylinder("test_cylinder", modelPos["test_cylinder"],
      math::Vector3::Zero);
  SpawnBox("test_non_unit_box", modelSize["test_non_unit_box"],
      modelPos["test_non_unit_box"], math::Vector3::Zero);

  // spawn another set of shapes and use messages to resize these
  modelPos["test_box2"] = math::Vector3(0, 9, z0);
  modelPos["test_sphere2"] = math::Vector3(4, 9, z0);
  modelPos["test_cylinder2"] = math::Vector3(8, 9, z0);

  modelSize["test_box2"] = math::Vector3::One;
  modelSize["test_sphere2"] = math::Vector3::One;
  modelSize["test_cylinder2"] = math::Vector3::One;

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
  for (auto const &iter : modelPos)
  {
    std::string name = iter.first;
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
  for (auto const &iter : modelPos)
  {
    std::string name = iter.first;
    model = world->GetModel(name);
    if (*(name.rbegin()) == '2')
    {
      // Use a message to resize this one
      msgs::Model msg;
      msg.set_name(name);
      msg.set_id(model->GetId());
      msgs::Set(msg.mutable_scale(),
          scaleFactor * ignition::math::Vector3d::One);
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
  for (auto const &iter : modelPos)
  {
    std::string name = iter.first;
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

  // verify geom msgs contain resized values.
  for (auto const &iter : modelPos)
  {
    std::string name = iter.first;
    model = world->GetModel(name);
    msgs::Model modelMsg;
    model->FillMsg(modelMsg);

    EXPECT_EQ(msgs::ConvertIgn(modelMsg.scale()),
        scaleFactor * ignition::math::Vector3d::One);
    for (int i = 0; i < modelMsg.link_size(); ++i)
    {
      msgs::Link linkMsg = modelMsg.link(i);

      // verify visual geom msgs
      for (int j = 0; j < linkMsg.visual_size(); ++j)
      {
        msgs::Visual visualMsg = linkMsg.visual(j);
        msgs::Geometry geomMsg = visualMsg.geometry();
        if (geomMsg.has_box())
        {
          EXPECT_EQ(msgs::ConvertIgn(geomMsg.box().size()),
              modelSize[name].Ign() * scaleFactor);
        }
        else if (geomMsg.has_sphere())
        {
          EXPECT_DOUBLE_EQ(geomMsg.sphere().radius(),
              modelSize[name].x * 0.5 * scaleFactor);
        }
        else if (geomMsg.has_cylinder())
        {
          EXPECT_DOUBLE_EQ(geomMsg.cylinder().radius(),
              modelSize[name].x * 0.5 * scaleFactor);
          EXPECT_DOUBLE_EQ(geomMsg.cylinder().length(),
              modelSize[name].z * scaleFactor);
        }
      }

      // verify collision geom msgs
      for (int j = 0; j < linkMsg.collision_size(); ++j)
      {
        msgs::Collision collisionMsg = linkMsg.collision(j);
        msgs::Geometry geomMsg = collisionMsg.geometry();
        if (geomMsg.has_box())
        {
          EXPECT_EQ(msgs::ConvertIgn(geomMsg.box().size()),
              modelSize[name].Ign() * scaleFactor);
        }
        else if (geomMsg.has_sphere())
        {
          EXPECT_DOUBLE_EQ(geomMsg.sphere().radius(),
              modelSize[name].x * 0.5 * scaleFactor);
        }
        else if (geomMsg.has_cylinder())
        {
          EXPECT_DOUBLE_EQ(geomMsg.cylinder().radius(),
              modelSize[name].x * 0.5 * scaleFactor);
          EXPECT_DOUBLE_EQ(geomMsg.cylinder().length(),
              modelSize[name].z * scaleFactor);
        }
      }
    }
  }

  // verify geom sdfs contain resized values.
  for (auto const &iter : modelPos)
  {
    std::string name = iter.first;
    model = world->GetModel(name);
    sdf::ElementPtr modelElem = model->GetSDF();

    EXPECT_TRUE(modelElem->HasElement("link"));
    sdf::ElementPtr linkElem = modelElem->GetElement("link");
    while (linkElem)
    {
      // verify visual geom sdf
      EXPECT_TRUE(linkElem->HasElement("visual"));
      sdf::ElementPtr visualElem = linkElem->GetElement("visual");

      EXPECT_TRUE(visualElem->HasElement("geometry"));
      sdf::ElementPtr visualGeomElem = visualElem->GetElement("geometry");
      if (visualGeomElem->HasElement("box"))
      {
        sdf::ElementPtr boxElem = visualGeomElem->GetElement("box");
        math::Vector3 size = boxElem->Get<math::Vector3>("size");
        EXPECT_EQ(size, modelSize[name] * scaleFactor);
      }
      else if (visualGeomElem->HasElement("sphere"))
      {
        sdf::ElementPtr sphereElem = visualGeomElem->GetElement("sphere");
        double radius = sphereElem->Get<double>("radius");
        EXPECT_EQ(radius, modelSize[name].x * 0.5 * scaleFactor);
      }
      else if (visualGeomElem->HasElement("cylinder"))
      {
        sdf::ElementPtr cylinderElem = visualGeomElem->GetElement("cylinder");
        double radius = cylinderElem->Get<double>("radius");
        EXPECT_EQ(radius, modelSize[name].x * 0.5 * scaleFactor);
        double length = cylinderElem->Get<double>("length");
        EXPECT_EQ(length, modelSize[name].z * scaleFactor);
      }

      // verify collision geom sdf
      EXPECT_TRUE(linkElem->HasElement("collision"));
      sdf::ElementPtr collisionElem = linkElem->GetElement("collision");

      EXPECT_TRUE(collisionElem->HasElement("geometry"));
      sdf::ElementPtr collisionGeomElem = collisionElem->GetElement("geometry");
      if (collisionGeomElem->HasElement("box"))
      {
        sdf::ElementPtr boxElem = collisionGeomElem->GetElement("box");
        math::Vector3 size = boxElem->Get<math::Vector3>("size");
        EXPECT_EQ(size, modelSize[name] * scaleFactor);
      }
      else if (collisionGeomElem->HasElement("sphere"))
      {
        sdf::ElementPtr sphereElem = collisionGeomElem->GetElement("sphere");
        double radius = sphereElem->Get<double>("radius");
        EXPECT_EQ(radius, modelSize[name].x * 0.5 * scaleFactor);
      }
      else if (collisionGeomElem->HasElement("cylinder"))
      {
        sdf::ElementPtr cylinderElem =
            collisionGeomElem->GetElement("cylinder");
        double radius = cylinderElem->Get<double>("radius");
        EXPECT_EQ(radius, modelSize[name].x * 0.5 * scaleFactor);
        double length = cylinderElem->Get<double>("length");
        EXPECT_EQ(length, modelSize[name].z * scaleFactor);
      }
      linkElem = linkElem->GetNextElement("link");
    }
  }
}

////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::LinkVisualMsg(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::ostringstream sdfStream;
  math::Pose pose(0, 0, 0, 0, 0, 0);
  math::Vector3 boxSize(1, 1, 1);
  sdfStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='box_test'>"
    << "<allow_auto_disable>false</allow_auto_disable>"
    << "<link name ='body'>"
    << "  <pose>" << pose << "</pose>"
    << "  <inertial>"
    << "    <mass>4.0</mass>"
    << "    <inertia>"
    << "      <ixx>0.1667</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>"
    << "      <iyy>0.1667</iyy> <iyz>0.0</iyz>"
    << "      <izz>0.1667</izz>"
    << "    </inertia>"
    << "  </inertial>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>" << boxSize << "</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <box><size>" << boxSize << "</size></box>"
    << "    </geometry>"
    << "  </visual>"
    << "  <visual name ='visual2'>"
    << "    <geometry>"
    << "      <box><size>" << boxSize << "</size></box>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(sdfStream.str());

  physics::ModelPtr model;
  model = world->GetModel("box_test");
  EXPECT_TRUE(model != NULL);
  msgs::Model msg;
  model->FillMsg(msg);

  EXPECT_EQ(msg.link_size(), 1);
  msgs::Link linkMsg = msg.link(0);

  // 3 visuals to be created: 1 link body + 2 visuals
  EXPECT_EQ(linkMsg.visual_size(), 3);

  // verify link body visual
  msgs::Visual visualMsg = linkMsg.visual(0);
  EXPECT_EQ(visualMsg.name(), "box_test::body");
  EXPECT_TRUE(visualMsg.has_type());
  EXPECT_EQ(visualMsg.type(), msgs::Visual::LINK);
  EXPECT_FALSE(visualMsg.has_geometry());

  // verify remaining visual msgs
  for (int i = 1; i < linkMsg.visual_size(); ++i)
  {
    msgs::Visual visualMsg = linkMsg.visual(i);
    std::stringstream visName;
    visName << "box_test::body::visual";
    if (i > 1)
      visName << i;
    EXPECT_EQ(visualMsg.name(), visName.str());
    EXPECT_TRUE(visualMsg.has_type());
    EXPECT_EQ(visualMsg.type(), msgs::Visual::VISUAL);
    msgs::Geometry geomMsg = visualMsg.geometry();
    EXPECT_TRUE(geomMsg.has_box());
    EXPECT_EQ(msgs::ConvertIgn(geomMsg.box().size()), boxSize.Ign());
  }
}

////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::JointMsg(const std::string &_physicsEngine)
{
  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::ostringstream sdfStream;
  sdfStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='joint_msg_test'>"
    << "<link name ='link1'>"
    << "  <pose>0 0 0 0 0 0</pose>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>1 1 1</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "</link>"
    << "<link name ='link2'>"
    << "  <pose>0 1 0 0 0 0</pose>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>1 1 1</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "</link>"
    << "<link name ='link3'>"
    << "  <pose>0 0 1 0 0 0</pose>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>1 1 1</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "</link>"
    << "<joint name='revolute_joint' type='revolute'>"
    << "  <parent>link1</parent>"
    << "  <child>link2</child>"
    << "  <pose>0 0 0 0 0 0</pose>"
    << "  <axis>"
    << "    <xyz>1 0 0</xyz>"
    << "    <use_parent_model_frame>0</use_parent_model_frame>"
    << "    <limit>"
    << "      <lower>-1</lower>"
    << "      <upper>1</upper>"
    << "      <effort>-1</effort>"
    << "      <velocity>-1</velocity>"
    << "    </limit>"
    << "    <dynamics>"
    << "      <damping>0.2</damping>"
    << "      <friction>0.1</friction>"
    << "    </dynamics>"
    << "  </axis>"
    << "</joint>"
    << "<joint name='screw_joint' type='screw'>"
    << "  <parent>link2</parent>"
    << "  <child>link3</child>"
    << "  <pose>0 0.2 0 0 0 0</pose>"
    << "  <axis>"
    << "    <xyz>0 1 0</xyz>"
    << "    <use_parent_model_frame>0</use_parent_model_frame>"
    << "    <limit>"
    << "      <lower>-2</lower>"
    << "      <upper>2</upper>"
    << "      <effort>-0.7</effort>"
    << "      <velocity>-1</velocity>"
    << "    </limit>"
    << "    <dynamics>"
    << "      <damping>0.3</damping>"
    << "      <friction>0.2</friction>"
    << "    </dynamics>"
    << "  </axis>"
    << "  <thread_pitch>2</thread_pitch>"
    << "</joint>"
    << "<joint name='gearbox_joint' type='gearbox'>"
    << "  <parent>link3</parent>"
    << "  <child>link1</child>"
    << "  <pose>0 0 0.1 0 0 0</pose>"
    << "  <axis>"
    << "    <xyz>1 0 0</xyz>"
    << "    <use_parent_model_frame>0</use_parent_model_frame>"
    << "    <limit>"
    << "      <lower>-1e6</lower>"
    << "      <upper>1e6</upper>"
    << "      <effort>-0.9</effort>"
    << "      <velocity>-0.1</velocity>"
    << "    </limit>"
    << "    <dynamics>"
    << "      <damping>0.4</damping>"
    << "      <friction>0.11</friction>"
    << "    </dynamics>"
    << "  </axis>"
    << "  <axis2>"
    << "    <xyz>0 0 1</xyz>"
    << "    <use_parent_model_frame>1</use_parent_model_frame>"
    << "    <limit>"
    << "      <lower>-1e3</lower>"
    << "      <upper>1e3</upper>"
    << "      <effort>-0.8</effort>"
    << "      <velocity>-0.2</velocity>"
    << "    </limit>"
    << "    <dynamics>"
    << "      <damping>0.23</damping>"
    << "      <friction>0.32</friction>"
    << "    </dynamics>"
    << "  </axis2>"
    << "  <gearbox_ratio>6.6</gearbox_ratio>"
    << "  <gearbox_reference_body>link_2</gearbox_reference_body>"
    << "</joint>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(sdfStream.str());

  physics::ModelPtr model;
  model = world->GetModel("joint_msg_test");
  EXPECT_TRUE(model != NULL);
  msgs::Model msg;
  model->FillMsg(msg);

  // only ode supports gearbox joint
  int jointSize = 2;
  if (_physicsEngine == "ode")
    jointSize = 3;

  EXPECT_EQ(msg.joint_size(), jointSize);

  {
    msgs::Joint jointMsg = msg.joint(0);
    EXPECT_EQ(jointMsg.name(), "joint_msg_test::revolute_joint");
    EXPECT_EQ(jointMsg.parent(), "joint_msg_test::link1");
    EXPECT_EQ(jointMsg.child(), "joint_msg_test::link2");
    EXPECT_EQ(msgs::ConvertIgn(jointMsg.pose()),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
    EXPECT_TRUE(jointMsg.has_axis1());
    EXPECT_TRUE(!jointMsg.has_axis2());
    msgs::Axis axis1Msg = jointMsg.axis1();
    EXPECT_EQ(msgs::ConvertIgn(axis1Msg.xyz()),
        ignition::math::Vector3d(1, 0, 0));
    EXPECT_EQ(axis1Msg.use_parent_model_frame(), false);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_lower(), -1);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_upper(), 1);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_effort(), -1);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_velocity(), -1);
    EXPECT_DOUBLE_EQ(axis1Msg.damping(), 0.2);
    // only ode and bullet return correct hinge friction param value
    if (_physicsEngine == "ode" || _physicsEngine == "bullet")
      EXPECT_DOUBLE_EQ(axis1Msg.friction(), 0.1);
  }

  {
    msgs::Joint jointMsg = msg.joint(1);
    EXPECT_EQ(jointMsg.name(), "joint_msg_test::screw_joint");
    EXPECT_EQ(jointMsg.parent(), "joint_msg_test::link2");
    EXPECT_EQ(jointMsg.child(), "joint_msg_test::link3");
    EXPECT_EQ(msgs::ConvertIgn(jointMsg.pose()),
        ignition::math::Pose3d(0, 0.2, 0, 0, 0, 0));
    EXPECT_TRUE(jointMsg.has_axis1());
    EXPECT_TRUE(jointMsg.has_axis2());
    msgs::Axis axis1Msg = jointMsg.axis1();
    EXPECT_EQ(msgs::ConvertIgn(axis1Msg.xyz()),
        ignition::math::Vector3d(0, 1, 0));
    EXPECT_EQ(axis1Msg.use_parent_model_frame(), false);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_lower(), -2);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_upper(), 2);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_effort(), -0.7);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_velocity(), -1);
    EXPECT_DOUBLE_EQ(axis1Msg.damping(), 0.3);
    // only ode returns correct screw friction param value
    if (_physicsEngine == "ode")
      EXPECT_DOUBLE_EQ(axis1Msg.friction(), 0.2);
    msgs::Joint::Screw screwMsg = jointMsg.screw();
    EXPECT_DOUBLE_EQ(screwMsg.thread_pitch(), 2);
  }

  // only ode supports gearbox joints
  if (_physicsEngine == "bullet" || _physicsEngine == "simbody"
      || _physicsEngine == "dart")
    return;

  {
    msgs::Joint jointMsg = msg.joint(2);
    EXPECT_EQ(jointMsg.name(), "joint_msg_test::gearbox_joint");
    EXPECT_EQ(jointMsg.parent(), "joint_msg_test::link3");
    EXPECT_EQ(jointMsg.child(), "joint_msg_test::link1");
    EXPECT_EQ(msgs::ConvertIgn(jointMsg.pose()),
        ignition::math::Pose3d(0, 0, 0.1, 0, 0, 0));
    EXPECT_TRUE(jointMsg.has_axis1());
    EXPECT_TRUE(jointMsg.has_axis2());
    msgs::Axis axis1Msg = jointMsg.axis1();
    EXPECT_EQ(msgs::ConvertIgn(axis1Msg.xyz()),
        ignition::math::Vector3d(1, 0, 0));
    EXPECT_EQ(axis1Msg.use_parent_model_frame(), false);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_lower(), -1e6);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_upper(), 1e6);
    EXPECT_DOUBLE_EQ(axis1Msg.limit_effort(), -0.9);
    EXPECT_EQ(axis1Msg.limit_velocity(), -0.1);
    EXPECT_DOUBLE_EQ(axis1Msg.damping(), 0.4);
    // gearbox friction param does not return correct value
    // EXPECT_DOUBLE_EQ(axis1Msg.friction(), 0.11);
    msgs::Axis axis2Msg = jointMsg.axis2();
    EXPECT_EQ(msgs::ConvertIgn(axis2Msg.xyz()),
        ignition::math::Vector3d(0, 0, 1));
    EXPECT_EQ(axis2Msg.use_parent_model_frame(), true);
    EXPECT_DOUBLE_EQ(axis2Msg.limit_lower(), -1e3);
    EXPECT_DOUBLE_EQ(axis2Msg.limit_upper(), 1e3);
    EXPECT_DOUBLE_EQ(axis2Msg.limit_effort(), -0.8);
    EXPECT_DOUBLE_EQ(axis2Msg.limit_velocity(), -0.2);
    EXPECT_DOUBLE_EQ(axis2Msg.damping(), 0.23);
    // gearbox friction param does not return correct value
    // EXPECT_DOUBLE_EQ(axis2Msg.friction(), 0.32);
    msgs::Joint::Gearbox gearboxMsg = jointMsg.gearbox();
    EXPECT_EQ(gearboxMsg.gearbox_reference_body(),
         "link_2");
    EXPECT_DOUBLE_EQ(gearboxMsg.gearbox_ratio(), 6.6);
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

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, LinkVisualMsg)
{
  LinkVisualMsg(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, JointMsg)
{
  JointMsg(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
