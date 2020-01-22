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

#include "plugins/TrackedVehiclePlugin.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

using namespace gazebo;

class TrackedVehiclePluginTest : public ServerFixture
{
  public: TrackedVehiclePluginTest()
  {
    this->Load("worlds/empty.world", true);
    this->world = physics::get_world("default");
  }

  protected: physics::WorldPtr world;
};

/// \brief A mock class that also allows access to protected members of
///        TrackedVehiclePlugin.
class TestTrackedVehiclePlugin : public TrackedVehiclePlugin
{
  public: double lastLeftVelocity = 0.0;
  public: double lastRightVelocity = 0.0;

  protected: void UpdateTrackSurface() override
  {
    // Nothing needed to be done here.
  }

  public: void SetTrackVelocityImpl(double _left, double _right) override
  {
    // Just record the velocities that were set.
    this->lastLeftVelocity = _left;
    this->lastRightVelocity = _right;
  }

  public: void SetTrackVelocity(double _left, double _right) override
  {
    TrackedVehiclePlugin::SetTrackVelocity(_left, _right);
  }

  public: std::string GetRobotNamespace() override
  {
    return TrackedVehiclePlugin::GetRobotNamespace();
  }

  public: double GetSteeringEfficiency() override
  {
    return TrackedVehiclePlugin::GetSteeringEfficiency();
  }

  public: boost::optional<double> GetTrackMu() override
  {
    return TrackedVehiclePlugin::GetTrackMu();
  }

  public: boost::optional<double> GetTrackMu2() override
  {
    return TrackedVehiclePlugin::GetTrackMu2();
  }

  public: double GetTracksSeparation() override
  {
    return TrackedVehiclePlugin::GetTracksSeparation();
  }

  public: msgs::Vector2d lastTracksVelMsg;
  public: void OnTracksVelReceived(ConstVector2dPtr &_msg)
  {
    this->lastTracksVelMsg = *_msg;
  }
};

/// \brief Test that reading parameters from the &lt;plugin&gt; tag works.
TEST_F(TrackedVehiclePluginTest, Load)
{
  // Create the model
  this->SpawnEmptyLink("model");
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "  <plugin name='test' filename='notimportant'>"
    << "    <robot_namespace>testNamespace</robot_namespace>"
    << "    <tracks_separation>0.397</tracks_separation>"
    << "    <steering_efficiency>0.1</steering_efficiency>"
    << "    <track_mu2>1.0</track_mu2>"
    << "  </plugin>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != NULL);

  TestTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  EXPECT_EQ("testNamespace", plugin.GetRobotNamespace());
  EXPECT_EQ(0.397, plugin.GetTracksSeparation());
  EXPECT_EQ(0.1, plugin.GetSteeringEfficiency());
  EXPECT_FALSE(plugin.GetTrackMu().is_initialized());
  EXPECT_TRUE(plugin.GetTrackMu2().is_initialized());
  EXPECT_EQ(1.0, plugin.GetTrackMu2().get());
}

/// \brief Test that the plugin initializes the ~/tracks_speed publisher.
TEST_F(TrackedVehiclePluginTest, Init)
{
  // Create the model
  this->SpawnEmptyLink("model");
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != NULL);

  TestTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));

  auto const tracksVelTopic = "/gazebo/default/testNamespace/tracks_speed";

  EXPECT_FALSE(NULL != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));

  ASSERT_NO_THROW(plugin.Init());

  EXPECT_TRUE(NULL != transport::TopicManager::Instance()->FindPublication(
    tracksVelTopic));
}

/// \brief Test that speed is reset to 0 when calling Reset() and that all speed
///        changes are published to ~/tracks_speed.
TEST_F(TrackedVehiclePluginTest, ResetAndTrackVelocityPublishersWork)
{
  // Create the model
  this->SpawnEmptyLink("model");
  this->WaitUntilEntitySpawn("model", 10, 100);
  physics::ModelPtr model = this->world->ModelByName("model");

  std::ostringstream pluginStr;
  pluginStr << "<sdf version ='" << SDF_VERSION << "'>"
            << "<model name='model'>"
            << "  <plugin name='test' filename='notimportant'>"
            << "    <robot_namespace>testNamespace</robot_namespace>"
            << "    <max_linear_speed>2.0</max_linear_speed>"
            << "  </plugin>"
            << "</model>"
            << "</sdf>";

  sdf::SDFPtr pluginSDF(new sdf::SDF);
  pluginSDF->SetFromString(pluginStr.str());

  sdf::ElementPtr elem = pluginSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("plugin");
  ASSERT_TRUE(elem != NULL);

  TestTrackedVehiclePlugin plugin;
  ASSERT_NO_THROW(plugin.Load(model, elem));
  ASSERT_NO_THROW(plugin.Init());

  transport::SubscriberPtr tracksVelSub = this->node->Subscribe(
    "/gazebo/default/testNamespace/tracks_speed",
    &TestTrackedVehiclePlugin::OnTracksVelReceived,
    &plugin);

  // Test before reset behavior.

  plugin.lastTracksVelMsg.Clear();

  plugin.SetTrackVelocity(1.0, 2.0);

  // Test that SetTrackVelocityImpl() was called.
  EXPECT_DOUBLE_EQ(plugin.lastLeftVelocity, 1.0);
  EXPECT_DOUBLE_EQ(plugin.lastRightVelocity, 2.0);

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

  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.x(), 1.0);
  EXPECT_DOUBLE_EQ(plugin.lastTracksVelMsg.y(), 2.0);

  // Test after reset behavior.

  plugin.lastTracksVelMsg.Clear();

  plugin.Reset();

  // Test that SetTrackVelocityImpl() was called.
  EXPECT_DOUBLE_EQ(plugin.lastLeftVelocity, 0.0);
  EXPECT_DOUBLE_EQ(plugin.lastRightVelocity, 0.0);

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

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
