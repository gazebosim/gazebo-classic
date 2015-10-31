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
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class InertiaMsgsTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Set inertia parameters over ~/model/modify
  /// and verify that Inertial accessors register the change.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void InertialAccessors(const std::string &_physicsEngine);

  /// \brief Set center of mass of link over ~/model/modify
  /// and verify that it causes a seesaw to unbalance.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetCoG(const std::string &_physicsEngine);

  /// \brief Set mass of link over ~/model/modify
  /// and verify that it causes a seesaw to unbalance.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetMass(const std::string &_physicsEngine);

  /// \brief Set moment of inertia of pendulums over ~/model/modify
  /// and verify that it changes frequency of oscillation.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SetPendulumInertia(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void InertiaMsgsTest::InertialAccessors(const std::string &_physicsEngine)
{
  Load("worlds/seesaw.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string modelName("cube1");
  auto model = world->GetModel(modelName);
  ASSERT_TRUE(model != NULL);
  auto link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  auto inertial = link->GetInertial();
  ASSERT_TRUE(inertial != NULL);
  const double mass = inertial->GetMass();
  const math::Vector3 cog = inertial->GetCoG();
  const math::Vector3 Ixxyyzz = inertial->GetPrincipalMoments();
  const math::Vector3 Ixyxzyz = inertial->GetProductsofInertia();
  EXPECT_DOUBLE_EQ(mass, 45.56250000000001);
  EXPECT_EQ(cog, math::Vector3::Zero);
  EXPECT_EQ(Ixxyyzz, 1.537734375*math::Vector3::One);
  EXPECT_EQ(Ixyxzyz, math::Vector3::Zero);

  // new inertial values
  msgs::Model msg;
  msg.set_name(modelName);
  msg.add_link();
  auto msgLink = msg.mutable_link(0);
  msgLink->set_name("link");
  msgLink->set_id(link->GetId());
  auto msgInertial = msgLink->mutable_inertial();
  msgInertial->set_mass(99.9);
  msgInertial->set_ixx(12.3);
  msgInertial->set_ixy(0.123);
  msgInertial->set_ixz(0.456);
  msgInertial->set_iyy(13.4);
  msgInertial->set_iyz(0.789);
  msgInertial->set_izz(15.6);
  const ignition::math::Vector3d newCog(1.1, -2.2, 3.3);
  msgs::Set(msgInertial->mutable_pose(), ignition::math::Pose3d(
    newCog, ignition::math::Quaterniond()));

  // Set inertial properties by publishing to "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");
  modelPub->WaitForConnection();
  modelPub->Publish(msg, true);

  while (newCog != inertial->GetCoG().Ign())
  {
    world->Step(1);
    common::Time::MSleep(1);
    modelPub->Publish(msg, true);
  }
  EXPECT_DOUBLE_EQ(inertial->GetMass(), msgInertial->mass());
  EXPECT_EQ(inertial->GetCoG().Ign(), newCog);
  EXPECT_EQ(inertial->GetPrincipalMoments(),
            ignition::math::Vector3d(
                msgInertial->ixx(),
                msgInertial->iyy(),
                msgInertial->izz()));
  EXPECT_EQ(inertial->GetProductsofInertia(),
            ignition::math::Vector3d(
                msgInertial->ixy(),
                msgInertial->ixz(),
                msgInertial->iyz()));
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, InertialAccessors)
{
  InertialAccessors(GetParam());
}

/////////////////////////////////////////////////
void InertiaMsgsTest::SetCoG(const std::string &_physicsEngine)
{
  Load("worlds/seesaw.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0, 0, -9.8));

  const std::string modelName("plank");
  auto model = world->GetModel(modelName);
  ASSERT_TRUE(model != NULL);
  auto link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  auto inertial = link->GetInertial();
  ASSERT_TRUE(inertial != NULL);
  const double mass = inertial->GetMass();
  const math::Vector3 cog = inertial->GetCoG();
  const math::Vector3 Ixxyyzz = inertial->GetPrincipalMoments();
  const math::Vector3 Ixyxzyz = inertial->GetProductsofInertia();
  EXPECT_DOUBLE_EQ(mass, 120);
  EXPECT_EQ(cog, math::Vector3::Zero);
  EXPECT_EQ(Ixxyyzz, math::Vector3(2.564, 360.064, 362.5));
  EXPECT_EQ(Ixyxzyz, math::Vector3::Zero);

  // new center of mass
  msgs::Model msg;
  msg.set_name(modelName);
  msg.add_link();
  auto msgLink = msg.mutable_link(0);
  msgLink->set_name("link");
  msgLink->set_id(link->GetId());
  auto msgInertial = msgLink->mutable_inertial();
  const ignition::math::Vector3d newCoG(2.5, 0, 0);
  msgs::Set(msgInertial->mutable_pose(), ignition::math::Pose3d(
      newCoG, ignition::math::Quaterniond()));

  // Set inertial properties by publishing to "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");
  modelPub->WaitForConnection();
  modelPub->Publish(msg, true);

  while (newCoG != inertial->GetCoG().Ign())
  {
    world->Step(1);
    common::Time::MSleep(1);
    modelPub->Publish(msg, true);
  }
  EXPECT_EQ(inertial->GetCoG().Ign(), newCoG);

  world->Step(1000);
  EXPECT_GT(model->GetWorldPose().rot.GetAsEuler().y, 0.25);
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, SetCoG)
{
  std::string physicsEngine = GetParam();
  if (physicsEngine == "bullet" || physicsEngine == "simbody")
  {
    gzerr << physicsEngine
          << " doesn't yet support dynamically changing a link's center of mass"
          << std::endl;
    return;
  }
  SetCoG(GetParam());
}

/////////////////////////////////////////////////
void InertiaMsgsTest::SetMass(const std::string &_physicsEngine)
{
  Load("worlds/seesaw.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0, 0, -9.8));

  const std::string modelName("cube1");
  auto model = world->GetModel(modelName);
  ASSERT_TRUE(model != NULL);
  auto link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  auto inertial = link->GetInertial();
  ASSERT_TRUE(inertial != NULL);
  const double mass = inertial->GetMass();
  const math::Vector3 cog = inertial->GetCoG();
  const math::Vector3 Ixxyyzz = inertial->GetPrincipalMoments();
  const math::Vector3 Ixyxzyz = inertial->GetProductsofInertia();
  EXPECT_DOUBLE_EQ(mass, 45.56250000000001);
  EXPECT_EQ(cog, math::Vector3::Zero);
  EXPECT_EQ(Ixxyyzz, 1.537734375*math::Vector3::One);
  EXPECT_EQ(Ixyxzyz, math::Vector3::Zero);

  // new inertial values
  msgs::Model msg;
  msg.set_name(modelName);
  msg.add_link();
  auto msgLink = msg.mutable_link(0);
  msgLink->set_name("link");
  msgLink->set_id(link->GetId());
  auto msgInertial = msgLink->mutable_inertial();
  const double newMass = 500;
  msgInertial->set_mass(newMass);

  // Set inertial properties by publishing to "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");
  modelPub->WaitForConnection();
  modelPub->Publish(msg, true);

  while (!math::equal(newMass, inertial->GetMass()))
  {
    world->Step(1);
    common::Time::MSleep(1);
    modelPub->Publish(msg, true);
  }
  EXPECT_DOUBLE_EQ(inertial->GetMass(), msgInertial->mass());

  world->Step(1000);
  EXPECT_LT(model->GetWorldPose().pos.z, 0.40);
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, SetMass)
{
  std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody")
  {
    gzerr << physicsEngine
          << " doesn't yet support dynamically changing a link's mass"
          << std::endl;
    return;
  }
  SetMass(physicsEngine);
}

/////////////////////////////////////////////////
void InertiaMsgsTest::SetPendulumInertia(const std::string &_physicsEngine)
{
  Load("worlds/pendulum_axes.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0, 0, -9.8));
  double dt = physics->GetMaxStepSize();
  EXPECT_NEAR(dt, 1e-3, 1e-6);

  const std::vector<std::string> modelNames = {
    "pendulum_x_axis",
    "pendulum_y_axis",
  };
  std::vector<physics::ModelPtr> models;
  std::vector<physics::JointPtr> joints;
  std::vector<double> pendulumLengths;
  std::vector<double> initialAngles;
  std::vector<double> cycleAngles;
  std::vector<int> cycleCount;

  for (auto const &modelName : modelNames)
  {
    gzdbg << "Initializing model "
          << modelName
          << std::endl;

    auto model = world->GetModel(modelName);
    ASSERT_TRUE(model != NULL);
    models.push_back(model);

    auto link = model->GetLink();
    ASSERT_TRUE(link != NULL);

    auto joint = model->GetJoint("joint");
    ASSERT_TRUE(joint != NULL);
    joints.push_back(joint);

    // Compute distance from cg to joint anchor
    auto linkPose = link->GetWorldCoGPose();
    auto jointPose = joint->GetWorldPose();
    auto jointToCoG = linkPose.pos - jointPose.pos;
    double length = jointToCoG.GetLength();
    EXPECT_NEAR(length, 0.05, 1e-6);
    pendulumLengths.push_back(length);

    double angle =
      asin(jointToCoG.Cross(g).Dot(joint->GetGlobalAxis(0)) / length / 9.8);
    EXPECT_NEAR(angle, -M_PI / 10, 1e-6);
    initialAngles.push_back(angle);

    // hysteresis threshhold for cycle counting
    cycleAngles.push_back(angle / 2);

    // counting oscillation cycles
    cycleCount.push_back(0);
  }

  // unthrottle physics to allow for many timesteps
  physics->SetRealTimeUpdateRate(0.0);
  const int steps = 30000;
  const double timeStepped = steps * dt;
  for (int step = 0; step < steps; ++step)
  {
    world->Step(1);
    for (unsigned int i = 0; i < models.size(); ++i)
    {
      auto model = models[i];
      auto joint = joints[i];
      auto initialAngle = initialAngles[i];
      auto cycleAngle = cycleAngles[i];

      auto angle = joint->GetAngle(0).Radian() - initialAngle;
      if (angle / cycleAngle >= 1)
      {
        cycleAngles[i] *= -1;
        cycleCount[i]++;
      }
    }
  }

  for (unsigned int i = 0; i < models.size(); ++i)
  {
    auto length = pendulumLengths[i];
    auto cycles = cycleCount[i];
    // expected natural frequency for box pendulum (Hz)
    // write up the equations
    double freq = sqrt(300.0 / 401.0 * g.GetLength() / length) / 2 * M_1_PI;
    // 2 cycles counted per oscillation
    double expectedCycles = 2 * freq * timeStepped;
    EXPECT_EQ(cycles, int(expectedCycles));
  }

  // const std::string modelName("cube1");
  // auto model = world->GetModel(modelName);
  // ASSERT_TRUE(model != NULL);
  // auto link = model->GetLink();
  // ASSERT_TRUE(link != NULL);
  // auto inertial = link->GetInertial();
  // ASSERT_TRUE(inertial != NULL);
  // const double mass = inertial->GetMass();
  // const math::Vector3 cog = inertial->GetCoG();
  // const math::Vector3 Ixxyyzz = inertial->GetPrincipalMoments();
  // const math::Vector3 Ixyxzyz = inertial->GetProductsofInertia();
  // EXPECT_DOUBLE_EQ(mass, 45.56250000000001);
  // EXPECT_EQ(cog, math::Vector3::Zero);
  // EXPECT_EQ(Ixxyyzz, 1.537734375*math::Vector3::One);
  // EXPECT_EQ(Ixyxzyz, math::Vector3::Zero);

  // // new inertial values
  // msgs::Model msg;
  // msg.set_name(modelName);
  // msg.add_link();
  // auto msgLink = msg.mutable_link(0);
  // msgLink->set_name("link");
  // msgLink->set_id(link->GetId());
  // auto msgInertial = msgLink->mutable_inertial();
  // const double newMass = 500;
  // msgInertial->set_mass(newMass);

  // // Set inertial properties by publishing to "~/model/modify"
  // transport::PublisherPtr modelPub =
  //   this->node->Advertise<msgs::Model>("~/model/modify");
  // modelPub->WaitForConnection();
  // modelPub->Publish(msg, true);

  // while (!math::equal(newMass, inertial->GetMass()))
  // {
  //   world->Step(1);
  //   common::Time::MSleep(1);
  //   modelPub->Publish(msg, true);
  // }
  // EXPECT_DOUBLE_EQ(inertial->GetMass(), msgInertial->mass());

  // world->Step(1000);
  // EXPECT_LT(model->GetWorldPose().pos.z, 0.40);
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, SetPendulumInertia)
{
  SetPendulumInertia(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, InertiaMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
