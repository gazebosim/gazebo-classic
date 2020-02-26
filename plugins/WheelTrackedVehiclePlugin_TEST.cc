/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "plugins/WheelTrackedVehiclePlugin.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

using namespace gazebo;

class WheelTrackedVehiclePluginTest : public ServerFixture
{
  public: explicit WheelTrackedVehiclePluginTest(
    const std::string &_physicsEngine = "")
  {
    this->Load("worlds/empty.world", true, _physicsEngine);
    this->world = physics::get_world("default");
  }

  protected: physics::WorldPtr world;
};

class WheelTrackedVehiclePluginTestParametrized
  : public WheelTrackedVehiclePluginTest,
    public ::testing::WithParamInterface<const char*>
{
  public: WheelTrackedVehiclePluginTestParametrized() :
    WheelTrackedVehiclePluginTest(GetParam())
  {
  }
};

/// \brief A mock class that also allows access to protected members of
///        TrackedVehiclePlugin.
class TestWheelTrackedVehiclePlugin : public WheelTrackedVehiclePlugin
{
  public: double lastLeftVelocity = 0.0;
  public: double lastRightVelocity = 0.0;

  public: boost::optional<double> lastTrackMu;
  public: boost::optional<double> lastTrackMu2;

  public: void SetTrackVelocityImpl(double _left, double _right) override
  {
    // Just record the velocities that were set.
    this->lastLeftVelocity = _left;
    this->lastRightVelocity = _right;
    WheelTrackedVehiclePlugin::SetTrackVelocityImpl(_left, _right);
  }

  protected: void UpdateTrackSurface() override
  {
    WheelTrackedVehiclePlugin::UpdateTrackSurface();

    this->lastTrackMu = this->GetTrackMu();
    this->lastTrackMu2 = this->GetTrackMu2();
  }

  public: void SetTrackVelocity(double _left, double _right) override
  {
    TrackedVehiclePlugin::SetTrackVelocity(_left, _right);
  }

  public: msgs::Vector2d lastTracksVelMsg;
  public: void OnTracksVelReceived(ConstVector2dPtr &_msg)
  {
    this->lastTracksVelMsg = *_msg;
  }
};

void AddWheel(msgs::Model &_model, const std::string &_name,
              const msgs::Link *_body, const double _x, const double _y)
{
  msgs::AddCylinderLink(_model, 1.0, 0.5, 1.0);
  auto link = _model.mutable_link(_model.link_size()-1);
  link->set_name(_name);
  link->mutable_collision(0)->set_name(_name + "_collision");
  link->mutable_visual(0)->set_name(_name + "_visual");

  auto joint = _model.add_joint();
  joint->set_name(_name + "_j");
  joint->set_type(msgs::Joint_Type::Joint_Type_REVOLUTE);
  joint->mutable_pose()->mutable_position()->set_x(_x);
  joint->mutable_pose()->mutable_position()->set_y(_y);
  joint->mutable_axis1()->mutable_xyz()->set_y(1.);
  joint->set_parent(_body->name());
  joint->set_child(link->name());
}

/// \brief Test that reading parameters from the &lt;plugin&gt; tag works.
TEST_F(WheelTrackedVehiclePluginTest, LoadCorrectModel)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1);
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "  <plugin name='test' filename='notimportant'>"
    << "    <left_joint>fl_wheel_j</left_joint>"
    << "    <left_joint>rl_wheel_j</left_joint>"
    << "    <right_joint>fr_wheel_j</right_joint>"
    << "    <right_joint>rr_wheel_j</right_joint>"
    << "  </plugin>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));
}

/// \brief Test that Load() fails when a referenced wheel joint is not found.
TEST_F(WheelTrackedVehiclePluginTest, LoadWheelNotFound)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  // AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1); // This wheel is missing.
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <left_joint>fl_wheel_j</left_joint>"
            << "    <left_joint>rl_wheel_j</left_joint>"
            << "    <right_joint>fr_wheel_j</right_joint>"
            << "    <right_joint>rr_wheel_j</right_joint>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that trying to use a non-revolute joint as a wheel fails.
TEST_F(WheelTrackedVehiclePluginTest, LoadRequiresRevoluteJoints)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1);
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);
  // Change the joint type to e.g. prismatic, so that it's not revolute.
  modelMsg.mutable_joint(modelMsg.joint_size()-1)->set_type(
    msgs::Joint_Type::Joint_Type_PRISMATIC);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <left_joint>fl_wheel_j</left_joint>"
            << "    <left_joint>rl_wheel_j</left_joint>"
            << "    <right_joint>fr_wheel_j</right_joint>"
            << "    <right_joint>rr_wheel_j</right_joint>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that Load() fails if there are less than 2 wheels specified.
TEST_F(WheelTrackedVehiclePluginTest, LoadAtLeastTwoWheels)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1);
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <left_joint>fl_wheel_j</left_joint>"
            // << "    <left_joint>rl_wheel_j</left_joint>"// The missing wheel.
            << "    <right_joint>fr_wheel_j</right_joint>"
            << "    <right_joint>rr_wheel_j</right_joint>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that the plugin initializes the ~/tracks_speed publisher,
///        propagates track friction coefficients (mu, mu2) to links and
///        that ODE is switched to the cone friction model.
TEST_P(WheelTrackedVehiclePluginTestParametrized, Init)
{
  auto physicsType = this->world->Physics()->GetType();
  if (physicsType == "bullet")
  {
    gzerr << "Aborting test for Bullet, see issue #1270.\n";
    return;
  }
  else if (physicsType == "simbody")
  {
    gzerr << "Aborting test for Simbody, it doesn't support multiple "
      "required functions: bounding boxes for collisions, friction params "
      "(#989, 'GetParam unrecognized parameter [friction]') and so on.\n";
    return;
  }
  else if (physicsType == "dart")
  {
    gzerr << "Aborting test for DART, it doesn't support multiple "
      "required functions: bounding boxes for collisions, fmax joint param.\n";
    return;
  }

  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1);
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <left_joint>fl_wheel_j</left_joint>"
            << "    <left_joint>rl_wheel_j</left_joint>"
            << "    <right_joint>fr_wheel_j</right_joint>"
            << "    <right_joint>rr_wheel_j</right_joint>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "    <track_mu>42.0</track_mu>"
            << "    <track_mu2>24.0</track_mu2>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  auto const tracksVelTopic = "/gazebo/default/testNamespace/tracks_speed";

  auto friction = model->GetLink("fl_wheel")->
    GetCollision("fl_wheel_collision")->GetSurface()->FrictionPyramid();

  EXPECT_FALSE(nullptr != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));
  EXPECT_FALSE(plugin.lastTrackMu.is_initialized());
  EXPECT_FALSE(plugin.lastTrackMu2.is_initialized());
  EXPECT_DOUBLE_EQ(friction->MuPrimary(), 1.0);
  EXPECT_DOUBLE_EQ(friction->MuSecondary(), 1.0);

  ASSERT_NO_THROW(plugin.Init());

  EXPECT_TRUE(nullptr != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));
  EXPECT_TRUE(plugin.lastTrackMu.is_initialized());
  EXPECT_DOUBLE_EQ(plugin.lastTrackMu.get(), 42.0);
  EXPECT_TRUE(plugin.lastTrackMu2.is_initialized());
  EXPECT_DOUBLE_EQ(plugin.lastTrackMu2.get(), 24.0);
  EXPECT_DOUBLE_EQ(friction->MuPrimary(), 42.0);
  EXPECT_DOUBLE_EQ(friction->MuSecondary(), 24.0);

  if (physicsType == "ode")
  {
    auto odePhysics = boost::dynamic_pointer_cast<physics::ODEPhysics>(
      this->world->Physics());
    ASSERT_NE(odePhysics, nullptr);

    EXPECT_EQ(odePhysics->GetFrictionModel(), "cone_model");
  }
}

/// \brief Test that speed is reset to 0 when calling Reset() and that all speed
///        changes are published to ~/tracks_speed.
TEST_P(WheelTrackedVehiclePluginTestParametrized,
       ResetAndTrackVelocityPublishersWork)
{
  auto physicsType = this->world->Physics()->GetType();
  if (physicsType == "bullet")
  {
    gzerr << "Aborting test for Bullet, see issue #1270.\n";
    return;
  }
  else if (physicsType == "simbody")
  {
    gzerr << "Aborting test for Simbody, the vehicle cannot steer there.\n";
    return;
  }
  else if (physicsType == "dart")
  {
    gzerr << "Aborting test for DART, the vehicle cannot steer there.\n";
    return;
  }

  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(1., 1., 1.));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddWheel(modelMsg, "fl_wheel", bodyLink, -1, 1);
  AddWheel(modelMsg, "fr_wheel", bodyLink, 1, 1);
  AddWheel(modelMsg, "rl_wheel", bodyLink, -1, -1);
  AddWheel(modelMsg, "rr_wheel", bodyLink, 1, -1);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <left_joint>fl_wheel_j</left_joint>"
            << "    <left_joint>rl_wheel_j</left_joint>"
            << "    <right_joint>fr_wheel_j</right_joint>"
            << "    <right_joint>rr_wheel_j</right_joint>"
            << "    <max_linear_speed>2.0</max_linear_speed>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "    <track_mu>2.0</track_mu>"
            << "    <track_mu2>0.5</track_mu2>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != nullptr);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != nullptr);

  TestWheelTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));
  ASSERT_NO_THROW(plugin.Init());

  transport::SubscriberPtr tracksVelSub = this->node->Subscribe(
    "/gazebo/default/testNamespace/tracks_speed",
    &TestWheelTrackedVehiclePlugin::OnTracksVelReceived,
    &plugin);

  // Test before reset behavior.

  plugin.lastTracksVelMsg.Clear();

  plugin.SetTrackVelocity(2.0, 2.0);
  this->world->Step(100);

  // Test that SetTrackVelocityImpl() was called.
  EXPECT_DOUBLE_EQ(plugin.lastLeftVelocity, 2.0);
  EXPECT_DOUBLE_EQ(plugin.lastRightVelocity, 2.0);
  // vel is the angular velocity, so it is different from the linear one
  EXPECT_NEAR(model->GetJoint("fl_wheel_j")->GetVelocity(0), 4.0, 0.25);
  EXPECT_NEAR(model->GetJoint("fr_wheel_j")->GetVelocity(0), 4.0, 0.25);

  // Test that the correct message was sent on ~/tracks_speed.
  {
    int waitCount = 0, maxWaitCount = 3000;
#if GOOGLE_PROTOBUF_VERSION < 3001000
    while (plugin.lastTracksVelMsg.ByteSize() == 0
        && ++waitCount < maxWaitCount)
#else
  // ByteSizeLong appeared in version 3.1 of Protobuf, and ByteSize
  // became deprecated.
    while (plugin.lastTracksVelMsg.ByteSizeLong() == 0
        && ++waitCount < maxWaitCount)
#endif
    {
      common::Time::MSleep(10);
    }
    ASSERT_LT(waitCount, maxWaitCount);
  }

  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.x(), 2.0);
  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.y(), 2.0);

  // Test after reset behavior.

  plugin.lastTracksVelMsg.Clear();

  plugin.Reset();
  this->world->Step(100);

  // Test that SetTrackVelocityImpl() was called.
  EXPECT_DOUBLE_EQ(plugin.lastLeftVelocity, 0.0);
  EXPECT_DOUBLE_EQ(plugin.lastRightVelocity, 0.0);
  EXPECT_NEAR(model->GetJoint("fl_wheel_j")->GetVelocity(0), 0.0, 1e-1);
  EXPECT_NEAR(model->GetJoint("fr_wheel_j")->GetVelocity(0), 0.0, 1e-1);

  // Test that the correct message was sent on ~/tracks_speed.
  {
    int waitCount = 0, maxWaitCount = 3000;
#if GOOGLE_PROTOBUF_VERSION < 3001000
    while (plugin.lastTracksVelMsg.ByteSize() == 0
        && ++waitCount < maxWaitCount)
#else
  // ByteSizeLong appeared in version 3.1 of Protobuf, and ByteSize
  // became deprecated.
    while (plugin.lastTracksVelMsg.ByteSizeLong() == 0
        && ++waitCount < maxWaitCount)
#endif
    {
      common::Time::MSleep(10);
    }
    ASSERT_LT(waitCount, maxWaitCount);
  }

  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.x(), 0.0);
  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.y(), 0.0);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines,
                        WheelTrackedVehiclePluginTestParametrized,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
