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

#include <stdint.h>

#include "plugins/SimpleTrackedVehiclePlugin.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

#include "gazebo/physics/ode/ODECollision.hh"

using namespace gazebo;

class SimpleTrackedVehiclePluginTest : public ServerFixture
{
  public: explicit SimpleTrackedVehiclePluginTest(
    const std::string &_physicsEngine = "")
  {
    this->Load("worlds/empty.world", true, _physicsEngine);
    this->world = physics::get_world("default");
  }

  protected: physics::WorldPtr world;
};

class SimpleTrackedVehiclePluginTestParametrized
  : public SimpleTrackedVehiclePluginTest,
    public ::testing::WithParamInterface<const char*>
{
  public: SimpleTrackedVehiclePluginTestParametrized() :
    SimpleTrackedVehiclePluginTest(GetParam())
  {
  }
};

/// \brief A mock class that also allows access to protected members of
///        TrackedVehiclePlugin.
class TestSimpleTrackedVehiclePlugin : public SimpleTrackedVehiclePlugin
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
    SimpleTrackedVehiclePlugin::SetTrackVelocityImpl(_left, _right);
  }

  public: double GetTrackVelocity(Tracks track) const
  {
    return this->trackVelocity.at(track);
  }

  protected: void UpdateTrackSurface() override
  {
    SimpleTrackedVehiclePlugin::UpdateTrackSurface();

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

  public: unsigned int GetCollideWithoutContactBitmask() const
  {
    return this->collideWithoutContactBitmask;
  }

  public:
  ignition::math::Vector3d ComputeFrictionDirectionPublic(
    const double _linearSpeed, const double _angularSpeed,
    const bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
    const ignition::math::Vector3d &_bodyYAxisGlobal,
    const ignition::math::Vector3d &_centerOfRotation,
    const ignition::math::Vector3d &_contactWorldPosition,
    const ignition::math::Vector3d &_contactNormal,
    const ignition::math::Vector3d &_beltDirection) const
  {
    return this->ComputeFrictionDirection(_linearSpeed, _angularSpeed,
        _drivingStraight, _bodyPose, _bodyYAxisGlobal, _centerOfRotation,
        _contactWorldPosition, _contactNormal, _beltDirection);
  }
};

msgs::Link* AddTrack(msgs::Model &_model, const std::string &_name,
              const msgs::Link *_body, const double _x, const double _y)
{
  msgs::AddBoxLink(_model, 1.0, ignition::math::Vector3d(0.1, 0.2, 0.5));
  auto link = _model.mutable_link(_model.link_size()-1);
  link->set_name(_name);
  link->mutable_collision(0)->set_name(_name + "_collision");
  link->mutable_visual(0)->set_name(_name + "_visual");
  link->mutable_pose()->mutable_position()->set_x(_x);
  link->mutable_pose()->mutable_position()->set_y(_y);

  auto joint = _model.add_joint();
  joint->set_name(_name + "_j");
  joint->set_type(msgs::Joint_Type::Joint_Type_FIXED);
  joint->set_parent(_body->name());
  joint->set_child(link->name());

  return link;
}

uint64_t GetCategoryBits(const std::string& _physicsEngine,
                         physics::ModelPtr _model,
                         const std::string& _linkName)
{
  if (_physicsEngine == "ode" || _physicsEngine == "")
  {
    auto odeCollision = boost::dynamic_pointer_cast<physics::ODECollision>(
      _model->GetLink(_linkName)->GetCollisions()[0]);
    auto odeID = odeCollision->GetCollisionId();
    return dGeomGetCategoryBits(odeID);
  }
  else
  {
    ADD_FAILURE() <<
      "Category bits can only be determined for ODE in this test.";
    return 0u;
  }
}

/// \brief Test that reading parameters from the &lt;plugin&gt; tag works.
TEST_F(SimpleTrackedVehiclePluginTest, LoadCorrectModel)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "  <plugin name='test' filename='notimportant'>"
    << "    <body>body</body>"
    << "    <left_track>left_track</left_track>"
    << "    <right_track>right_track</right_track>"
    << "    <collide_without_contact_bitmask>"
    << "      3"
    << "    </collide_without_contact_bitmask>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  ASSERT_EQ(plugin.GetCollideWithoutContactBitmask(), 3u);
}

/// \brief Test that Load() fails when a referenced track link is not found.
TEST_F(SimpleTrackedVehiclePluginTest, LoadTrackNotFound)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  //  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2); // Missing track.

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that Load() fails when the referenced body link is not found.
TEST_F(SimpleTrackedVehiclePluginTest, LoadBodyNotFound)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>wrong_body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that Load() fails when a track link is not specified.
TEST_F(SimpleTrackedVehiclePluginTest, LoadTrackNotSpecified)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            // << "    <left_track>left_track</left_track>"  // Missing spec.
            << "    <right_track>right_track</right_track>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that Load() fails when a body link is not specified.
TEST_F(SimpleTrackedVehiclePluginTest, LoadBodyNotSpecified)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            // << "    <body>body</body>"  // Missing spec.
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_THROW(plugin.Load(model, elem), std::runtime_error);
}

/// \brief Test that Load() doesn't fail when flippers are specified.
TEST_F(SimpleTrackedVehiclePluginTest, LoadFlippers)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  auto leftTrack = AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  auto rightTrack = AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);
  AddTrack(modelMsg, "front_left_flipper", leftTrack, 0.2, 0.2);
  AddTrack(modelMsg, "rear_left_flipper", leftTrack, -0.2, 0.2);
  AddTrack(modelMsg, "front_right_flipper", rightTrack, 0.2, -0.2);
  AddTrack(modelMsg, "rear_right_flipper", rightTrack, -0.2, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
            << "    <left_flipper>front_left_flipper</left_flipper>"
            << "    <left_flipper>rear_left_flipper</left_flipper>"
            << "    <right_flipper>front_right_flipper</right_flipper>"
            << "    <right_flipper>rear_right_flipper</right_flipper>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  ASSERT_EQ(plugin.GetNumTracks(Tracks::LEFT), 3u);
  ASSERT_EQ(plugin.GetNumTracks(Tracks::RIGHT), 3u);
}

/// \brief Test that Load() doesn't fail when flippers are specified a different
/// count on the left and right.
TEST_F(SimpleTrackedVehiclePluginTest, LoadFlippersAssymetric)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  auto leftTrack = AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  auto rightTrack = AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);
  AddTrack(modelMsg, "front_left_flipper", leftTrack, 0.2, 0.2);
  AddTrack(modelMsg, "rear_left_flipper", leftTrack, -0.2, 0.2);
  AddTrack(modelMsg, "front_right_flipper", rightTrack, 0.2, -0.2);
//  AddTrack(modelMsg, "rear_right_flipper", rightTrack, -0.2, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
            << "    <left_flipper>front_left_flipper</left_flipper>"
            << "    <left_flipper>rear_left_flipper</left_flipper>"
            << "    <right_flipper>front_right_flipper</right_flipper>"
//            << "    <right_flipper>rear_right_flipper</right_flipper>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  ASSERT_EQ(plugin.GetNumTracks(Tracks::LEFT), 3u);
  ASSERT_EQ(plugin.GetNumTracks(Tracks::RIGHT), 2u);
}

/// \brief Test that Load() doesn't fail when flippers are specified but a
/// referenced link does not exist. It should instead just ignore the
/// non-existing flipper link.
TEST_F(SimpleTrackedVehiclePluginTest, LoadFlippersMissingLink)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  auto leftTrack = AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  auto rightTrack = AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);
  AddTrack(modelMsg, "front_left_flipper", leftTrack, 0.2, 0.2);
  AddTrack(modelMsg, "rear_left_flipper", leftTrack, -0.2, 0.2);
  AddTrack(modelMsg, "front_right_flipper", rightTrack, 0.2, -0.2);
//  AddTrack(modelMsg, "rear_right_flipper", rightTrack, -0.2, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
            << "    <left_flipper>front_left_flipper</left_flipper>"
            << "    <left_flipper>rear_left_flipper</left_flipper>"
            << "    <right_flipper>front_right_flipper</right_flipper>"
            << "    <right_flipper>rear_right_flipper</right_flipper>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  ASSERT_EQ(plugin.GetNumTracks(Tracks::LEFT), 3u);
  ASSERT_EQ(plugin.GetNumTracks(Tracks::RIGHT), 2u);
}

/// \brief Test that the plugin initializes the ~/tracks_speed publisher and
///        propagates track friction coefficients (mu, mu2) to links.
TEST_P(SimpleTrackedVehiclePluginTestParametrized, Init)
{
  auto physicsType = this->world->Physics()->GetType();
  if (physicsType != "ode")
  {
    gzerr << "Aborting test for " << physicsType << ", this model is only "
      "available in ODE.\n";
    return;
  }

  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  auto leftFlipper = AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);
  AddTrack(modelMsg, "front_left_flipper", leftFlipper, 0.2, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
            << "    <left_flipper>front_left_flipper</left_flipper>"
            << "    <track_mu>42.0</track_mu>"
            << "    <track_mu2>24.0</track_mu2>"
            << "    <collide_without_contact_bitmask>"
            << "      3"
            << "    </collide_without_contact_bitmask>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  auto const tracksVelTopic = "/gazebo/default/testNamespace/tracks_speed";

  auto surface = model->GetLink("left_track")->
      GetCollisions()[0]->GetSurface();
  ASSERT_NE(surface, nullptr);
  auto friction = surface->FrictionPyramid();

  auto flipperSurface = model->GetLink("front_left_flipper")->
      GetCollisions()[0]->GetSurface();
  ASSERT_NE(flipperSurface, nullptr);
  auto flipperFriction = flipperSurface->FrictionPyramid();

  EXPECT_FALSE(nullptr != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));
  EXPECT_FALSE(plugin.lastTrackMu.is_initialized());
  EXPECT_FALSE(plugin.lastTrackMu2.is_initialized());
  EXPECT_DOUBLE_EQ(friction->MuPrimary(), 1.0);
  EXPECT_DOUBLE_EQ(friction->MuSecondary(), 1.0);
  EXPECT_DOUBLE_EQ(flipperFriction->MuPrimary(), 1.0);
  EXPECT_DOUBLE_EQ(flipperFriction->MuSecondary(), 1.0);

  ASSERT_NO_THROW(plugin.Init());

  EXPECT_TRUE(nullptr != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));
  EXPECT_TRUE(plugin.lastTrackMu.is_initialized());
  EXPECT_DOUBLE_EQ(plugin.lastTrackMu.get(), 42.0);
  EXPECT_TRUE(plugin.lastTrackMu2.is_initialized());
  EXPECT_DOUBLE_EQ(plugin.lastTrackMu2.get(), 24.0);
  EXPECT_DOUBLE_EQ(friction->MuPrimary(), 42.0);
  EXPECT_DOUBLE_EQ(friction->MuSecondary(), 24.0);
  EXPECT_DOUBLE_EQ(flipperFriction->MuPrimary(), 42.0);
  EXPECT_DOUBLE_EQ(flipperFriction->MuSecondary(), 24.0);

  EXPECT_NE(GetCategoryBits(physicsType, model, "body"),
            GetCategoryBits(physicsType, model, "left_track"));

  EXPECT_NE(GetCategoryBits(physicsType, model, "body"),
            GetCategoryBits(physicsType, model, "front_left_flipper"));

  EXPECT_EQ(GetCategoryBits(physicsType, model, "left_track"),
            GetCategoryBits(physicsType, model, "front_left_flipper"));

  EXPECT_NE(GetCategoryBits(physicsType, model, "right_track"),
            GetCategoryBits(physicsType, model, "left_track"));

  EXPECT_NE(GetCategoryBits(physicsType, model, "right_track"),
            GetCategoryBits(physicsType, model, "front_left_flipper"));

  EXPECT_EQ(surface->collideWithoutContactBitmask, 3u);
  EXPECT_EQ(flipperSurface->collideWithoutContactBitmask, 3u);
}

/// \brief Test that speed is reset to 0 when calling Reset() and that all speed
///        changes are published to ~/tracks_speed.
TEST_P(SimpleTrackedVehiclePluginTestParametrized,
       ResetAndTrackVelocityPublishersWork)
{
  auto physicsType = this->world->Physics()->GetType();
  if (physicsType != "ode")
  {
    gzerr << "Aborting test for " << physicsType << ", this model is only "
      "available in ODE.\n";
    return;
  }

  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
            << "    <track_mu>2.0</track_mu>"
            << "    <track_mu2>0.5</track_mu2>"
            << "    <max_linear_speed>2.0</max_linear_speed>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));
  ASSERT_NO_THROW(plugin.Init());

  transport::SubscriberPtr tracksVelSub = this->node->Subscribe(
    "/gazebo/default/testNamespace/tracks_speed",
    &TestSimpleTrackedVehiclePlugin::OnTracksVelReceived,
    &plugin);

  // Test before reset behavior.

  plugin.lastTracksVelMsg.Clear();

  plugin.SetTrackVelocity(2.0, 2.0);
  this->world->Step(100);

  // Test that SetTrackVelocityImpl() was called.
  EXPECT_DOUBLE_EQ(plugin.lastLeftVelocity, 2.0);
  EXPECT_DOUBLE_EQ(plugin.lastRightVelocity, 2.0);
  EXPECT_DOUBLE_EQ(plugin.GetTrackVelocity(Tracks::LEFT), 2.0);
  EXPECT_DOUBLE_EQ(plugin.GetTrackVelocity(Tracks::RIGHT), 2.0);

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
  EXPECT_DOUBLE_EQ(plugin.GetTrackVelocity(Tracks::LEFT), 0.0);
  EXPECT_DOUBLE_EQ(plugin.GetTrackVelocity(Tracks::RIGHT), 0.0);

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

/// \brief Test the ComputeFrictionDirection function.
TEST_F(SimpleTrackedVehiclePluginTest, ComputeFrictionDirection)
{
  // Create the model
  msgs::Model modelMsg;
  modelMsg.set_name("model");
  modelMsg.set_is_static(false);

  msgs::AddBoxLink(modelMsg, 10., ignition::math::Vector3d(0.5, 0.25, 0.14));
  auto bodyLink = modelMsg.mutable_link(modelMsg.link_size()-1);
  bodyLink->set_name("body");

  AddTrack(modelMsg, "left_track", bodyLink, 0, 0.2);
  AddTrack(modelMsg, "right_track", bodyLink, 0, -0.2);

  this->SpawnModel(modelMsg);
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <body>body</body>"
            << "    <left_track>left_track</left_track>"
            << "    <right_track>right_track</right_track>"
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

  TestSimpleTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));
  ASSERT_NO_THROW(plugin.Init());

  auto linearSpeed = 0.;
  auto angularSpeed = 0.;
  auto drivingStraight = true;
  auto bodyPose = ignition::math::Pose3d::Zero;
  auto bodyYAxisGlobal = ignition::math::Vector3d::UnitY;
  auto centerOfRotation = ignition::math::Vector3d::One * ignition::math::INF_D;
  auto beltDirection = ignition::math::Vector3d::UnitX;
  auto contactPose = ignition::math::Vector3d::Zero;
  auto contactNormal = ignition::math::Vector3d(0, 0, 1);

  ignition::math::Vector3<double> expectedDir;
  ignition::math::Quaterniond rotation;

  // Straight driving means the friction to always point forward, maybe tilted
  // up or down if the contact point is on a curved surface.
  auto frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  EXPECT_NEAR(frictionDirection.Distance(beltDirection), 0, 1e-6);

  // The direction doesn't depend on the desired velocities.
  linearSpeed = M_PI;
  angularSpeed = M_E;
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  EXPECT_NEAR(frictionDirection.Distance(beltDirection), 0, 1e-6);

  // Test with belt rotated by pi/4.
  rotation.Axis(ignition::math::Vector3d::UnitZ, M_PI_4);
  beltDirection = rotation * beltDirection;
  bodyYAxisGlobal = rotation * bodyYAxisGlobal;
  bodyPose.Rot() = rotation * bodyPose.Rot();
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  EXPECT_NEAR(frictionDirection.Distance(beltDirection), 0, 1e-6);

  // Test with contact normal pointing pi/4 backwards (i.e. contact on a plane
  // lifted by pi/4 up).
  bodyPose = ignition::math::Pose3d::Zero;
  bodyYAxisGlobal = ignition::math::Vector3d::UnitY;
  beltDirection = ignition::math::Vector3d::UnitX;
  contactNormal[0] = -cos(M_PI_4);
  contactNormal[2] = sin(M_PI_4);
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal,  beltDirection);
  expectedDir = ignition::math::Vector3d(cos(M_PI_4), 0, sin(M_PI_4));
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // Tests for non-straight drive.
  drivingStraight = false;
  centerOfRotation = ignition::math::Vector3d::UnitY;
  contactPose = ignition::math::Vector3d::Zero;
  contactNormal = ignition::math::Vector3d(0, 0, 1);
  linearSpeed = 1.0;
  angularSpeed = 1.0;
  bodyPose = ignition::math::Pose3d::Zero;
  bodyYAxisGlobal = ignition::math::Vector3d::UnitY;
  beltDirection = ignition::math::Vector3d::UnitX;

  // Friction direction on the COR-body line points forward...
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = ignition::math::Vector3d(1, 0, 0);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // ... and doesn't change with distance of the COR.
  centerOfRotation.Y(2.);
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = ignition::math::Vector3d(1, 0, 0);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // ... it points backwards if the desired linear speed is backwards.
  linearSpeed = -1.;
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = ignition::math::Vector3d(-1, 0, 0);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // Checks for contact points in front of the COR-body line.
  contactPose[0] = 1.;
  centerOfRotation = ignition::math::Vector3d::UnitY;
  linearSpeed = 1.;
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = ignition::math::Vector3d(cos(M_PI_4), sin(M_PI_4), 0);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // The direction changes if COR gets further...
  centerOfRotation.Y(2.);
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = ignition::math::Vector3d(cos(atan2(1, 2)), sin(atan2(1, 2)), 0);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // ... and is inverted when going backwards.
  linearSpeed = -1.;
  angularSpeed = -1.;
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  expectedDir = -expectedDir;
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);

  // ... and is not changed by translating the body.
  bodyPose.Pos().X(bodyPose.Pos().X() + 5.);
  centerOfRotation.X(centerOfRotation.X() + 5.);
  contactPose[0] += 5.;
  frictionDirection = plugin.ComputeFrictionDirectionPublic(
    linearSpeed, angularSpeed, drivingStraight, bodyPose, bodyYAxisGlobal,
    centerOfRotation, contactPose, contactNormal, beltDirection);
  EXPECT_NEAR(frictionDirection.Distance(expectedDir), 0, 1e-6);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines,
                        SimpleTrackedVehiclePluginTestParametrized,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
