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
#include <string.h>
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

const double g_tolerance = 1e-4;

class FactoryTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void BoxSdf(const std::string &_physicsEngine);
  public: void Box(const std::string &_physicsEngine);
  public: void Sphere(const std::string &_physicsEngine);
  public: void Cylinder(const std::string &_physicsEngine);

  /// \brief Test spawning an actor with skin only.
  /// \param[in] _physicsEngine Physics engine name
  public: void ActorSkinOnly(const std::string &_physicsEngine);

  /// \brief Test spawning an actor with skin and animation.
  /// \param[in] _physicsEngine Physics engine name
  public: void ActorSkinAnim(const std::string &_physicsEngine);

  /// \brief Test spawning an actor with skin and trajectory.
  /// \param[in] _physicsEngine Physics engine name
  public: void ActorSkinTrajectory(const std::string &_physicsEngine);

  /// \brief Test spawning an actor with link and trajectory.
  /// \param[in] _physicsEngine Physics engine name
  public: void ActorLinkTrajectory(const std::string &_physicsEngine);

  /// \brief Test spawning an actor with a plugin.
  public: void ActorPlugin();

  /// \brief Test spawning an actor with skin, animation and trajectory.
  /// \param[in] _physicsEngine Physics engine name
  public: void ActorAll(const std::string &_physicsEngine);

  public: void Clone(const std::string &_physicsEngine);
};


class LightFactoryTest : public ServerFixture
{
  /// \brief Constructor.
  public: LightFactoryTest():
    responseCbCalled(false), modelInfoCbCalled(false) {}

  /// \brief Response callback to get entity information.
  /// \param[in] _msg Message holding the requested information.
  public: void ResponseCb(ConstResponsePtr &_msg);

  /// \brief Model info callback to get model information.
  /// \param[in] _msg Message holding the model information.
  public: void ModelInfoCb(ConstModelPtr &_msg);

  /// \brief Light factory publisher.
  protected: transport::PublisherPtr lightFactoryPub;

  /// \brief Entity info request publisher.
  protected: transport::PublisherPtr requestPub;

  /// \brief Entity info response subscriber.
  protected: transport::SubscriberPtr responseSub;

  /// \brief True if the callback function was called.
  protected: bool responseCbCalled;

  /// \brief Response message holding the entity information.
  protected: msgs::Response resMsg;

  /// \brief Mutex for the variables accessed by the callback.
  protected: std::mutex resMutex;

  /// \brief Subscriber for model info.
  protected: transport::SubscriberPtr modelInfoSub;

  /// \brief Model message from ~/model/info
  protected: msgs::Model modMsg;

  /// \brief True if the callback for model info was called
  protected: bool modelInfoCbCalled;

  /// \brief Mutex for model info callback.
  protected: std::mutex modMutex;

  /// \brief Factory publisher.
  protected: transport::PublisherPtr factoryPub;
};

///////////////////////////////////////////////////
// Verify that sdf is retained by entities spawned
// via factory messages. A change between 1.6, 1.7
// caused entities to lose their sdf data
// (see issue #651)
void FactoryTest::BoxSdf(const std::string &_physicsEngine)
{
  ignition::math::Pose3d setPose;
  this->Load("worlds/empty.world", true, _physicsEngine);

  unsigned int entityCount = 6;

  for (unsigned int i = 0; i < entityCount; ++i)
  {
    std::ostringstream name;
    name << "test_box_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond::Identity);
    SpawnBox(name.str(), ignition::math::Vector3d(1, 1, 1), setPose.Pos(),
        setPose.Rot().Euler());
  }

  // This loop must be separate from the previous loop to cause
  // the failure.
  for (unsigned int i = 0; i < entityCount; ++i)
  {
    std::ostringstream name;
    name << "test_box_" << i;

    physics::ModelPtr model = this->GetModel(name.str());
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
  this->Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_box_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond::Identity);
    SpawnBox(name.str(), ignition::math::Vector3d(1, 1, 1), setPose.Pos(),
        setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(testPose.Pos().Equal(setPose.Pos(), 0.1));
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
  this->Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_sphere_" << i;
    setPose.Set(ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond::Identity);
    SpawnSphere(name.str(), setPose.Pos(), setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(testPose.Pos().Equal(setPose.Pos(), 0.1));
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
  this->Load("worlds/empty.world", true, _physicsEngine);

  for (unsigned int i = 0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_cylinder_" << i;
    setPose.Set(
        ignition::math::Vector3d(0, 0, i+0.5),
        ignition::math::Quaterniond::Identity);
    SpawnCylinder(name.str(), setPose.Pos(), setPose.Rot().Euler());
    testPose = EntityPose(name.str());
    EXPECT_TRUE(testPose.Pos().Equal(setPose.Pos(), 0.1));
  }
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, Cylinder)
{
  Cylinder(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::ActorSkinOnly(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::string skinFile("walk.dae");
  std::ostringstream actorStr;
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <skin>"
    << "    <filename>" << skinFile << "</filename>"
    << "  </skin>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  this->WaitUntilEntitySpawn(actorName, 300, 10);

  auto model = this->GetModel(actorName);
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  // Check skin is still there
  EXPECT_TRUE(sdf->HasElement("skin"));
  auto skinElem = sdf->GetElement("skin");
  ASSERT_TRUE(skinElem != nullptr);
  EXPECT_EQ(skinElem->GetElement("filename")->Get<std::string>(),
      skinFile);

  // Check the skin file was copied into the animation
  EXPECT_TRUE(sdf->HasElement("animation"));
  auto animationElem = sdf->GetElement("animation");
  ASSERT_TRUE(animationElem != nullptr);
  EXPECT_EQ(animationElem->GetElement("filename")->Get<std::string>(),
      skinFile);

  // Check a default script was added
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check links were added
  EXPECT_TRUE(sdf->HasElement("link"));

  auto linkElem = sdf->GetElement("link");
  EXPECT_TRUE(linkElem != nullptr);

  int linkCount = 0;
  while (linkElem)
  {
    linkCount++;
    linkElem = linkElem->GetNextElement("link");
  }
  EXPECT_EQ(linkCount, 32);

  // Check the skin animation was loaded as default
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_FALSE(skelAnims.empty());
  EXPECT_EQ(skelAnims.size(), 1u);
  EXPECT_TRUE(skelAnims["__default__"] != nullptr);
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, ActorSkinOnly)
{
  ActorSkinOnly(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::ActorSkinAnim(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::string skinFile("walk.dae");
  std::string animFile("moonwalk.dae");
  std::string animName("moonwalk_animation");
  std::ostringstream actorStr;
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <skin>"
    << "    <filename>" << skinFile << "</filename>"
    << "  </skin>"
    << "  <animation name='" << animName << "'>"
    << "    <filename>" << animFile << "</filename>"
    << "  </animation>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  this->WaitUntilEntitySpawn(actorName, 300, 10);

  auto model = this->GetModel(actorName);
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  // Check skin is still there
  EXPECT_TRUE(sdf->HasElement("skin"));
  auto skinElem = sdf->GetElement("skin");
  ASSERT_TRUE(skinElem != nullptr);
  EXPECT_EQ(skinElem->GetElement("filename")->Get<std::string>(),
      skinFile);

  // Check the animation is still there
  EXPECT_TRUE(sdf->HasElement("animation"));
  auto animationElem = sdf->GetElement("animation");
  ASSERT_TRUE(animationElem != nullptr);
  EXPECT_EQ(animationElem->GetElement("filename")->Get<std::string>(),
      animFile);

  // Check a default script was added
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check links were added
  EXPECT_TRUE(sdf->HasElement("link"));

  auto linkElem = sdf->GetElement("link");
  EXPECT_TRUE(linkElem != nullptr);

  int linkCount = 0;
  while (linkElem)
  {
    linkCount++;
    linkElem = linkElem->GetNextElement("link");
  }
  EXPECT_EQ(linkCount, 32);

  // Check the anim animation was loaded
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_FALSE(skelAnims.empty());
  EXPECT_EQ(skelAnims.size(), 1u);
  EXPECT_TRUE(skelAnims[animName] != nullptr);
}

/////////////////////////////////////////////////
void FactoryTest::ActorSkinTrajectory(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::string skinFile("run.dae");
  std::string animName("run_animation");
  std::ostringstream actorStr;
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <skin>"
    << "    <filename>" << skinFile << "</filename>"
    << "  </skin>"
    << "  <script>"
    << "    <trajectory id='0' type='" << animName << "'>"
    << "      <waypoint>"
    << "        <time>0.0</time>"
    << "        <pose>1 1 0 0 0 0</pose>"
    << "      </waypoint>"
    << "      <waypoint>"
    << "        <time>1.0</time>"
    << "        <pose>2 2 0 0 0 0</pose>"
    << "      </waypoint>"
    << "    </trajectory>"
    << "  </script>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  this->WaitUntilEntitySpawn(actorName, 300, 10);

  auto model = this->GetModel(actorName);
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  // Check skin is still there
  EXPECT_TRUE(sdf->HasElement("skin"));
  auto skinElem = sdf->GetElement("skin");
  ASSERT_TRUE(skinElem != nullptr);
  EXPECT_EQ(skinElem->GetElement("filename")->Get<std::string>(),
      skinFile);

  // Check the skin file and the trajectory type were copied into the animation
  EXPECT_TRUE(sdf->HasElement("animation"));
  auto animationElem = sdf->GetElement("animation");
  ASSERT_TRUE(animationElem != nullptr);
  EXPECT_EQ(animationElem->GetElement("filename")->Get<std::string>(),
      skinFile);
  EXPECT_EQ(animationElem->Get<std::string>("name"), animName);

  // Check the script is still there
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check links were added
  EXPECT_TRUE(sdf->HasElement("link"));

  auto linkElem = sdf->GetElement("link");
  EXPECT_TRUE(linkElem != nullptr);

  int linkCount = 0;
  while (linkElem)
  {
    linkCount++;
    linkElem = linkElem->GetNextElement("link");
  }
  EXPECT_EQ(linkCount, 32);

  // Check the anim animation was loaded
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_FALSE(skelAnims.empty());
  EXPECT_EQ(skelAnims.size(), 1u);
  EXPECT_TRUE(skelAnims[animName] != nullptr);
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, ActorSkinTrajectory)
{
  ActorSkinTrajectory(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::ActorLinkTrajectory(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::ostringstream actorStr;
  std::string animName("link_animation");
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <link name='link'>"
    << "    <visual name='visual'>"
    << "      <geometry>"
    << "        <box>"
    << "          <size>.2 .2 .2</size>"
    << "        </box>"
    << "      </geometry>"
    << "    </visual>"
    << "  </link>"
    << "  <script>"
    << "    <trajectory id='0' type='" << animName << "'>"
    << "      <waypoint>"
    << "        <time>0.0</time>"
    << "        <pose>1 1 0 0 0 0</pose>"
    << "      </waypoint>"
    << "      <waypoint>"
    << "        <time>1.0</time>"
    << "        <pose>2 2 0 0 0 0</pose>"
    << "      </waypoint>"
    << "    </trajectory>"
    << "  </script>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  int sleep = 0;
  int maxSleep = 10;
  auto model = this->GetModel(actorName);
  while (!model && sleep < maxSleep)
  {
    common::Time::MSleep(300);
    model = this->GetModel(actorName);
    ++sleep;
  }
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  // Check skin is still there
  EXPECT_FALSE(sdf->HasElement("skin"));

  // Check the skin file and the trajectory type were copied into the animation
  EXPECT_FALSE(sdf->HasElement("animation"));

  // Check the script is still there
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check there's only one link
  EXPECT_TRUE(sdf->HasElement("link"));

  auto linkElem = sdf->GetElement("link");
  EXPECT_TRUE(linkElem != nullptr);

  int linkCount = 0;
  while (linkElem)
  {
    linkCount++;
    linkElem = linkElem->GetNextElement("link");
  }
  EXPECT_EQ(linkCount, 1);

  // Check there's no animation
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_TRUE(skelAnims.empty());
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, ActorLinkTrajectory)
{
  ActorLinkTrajectory(GetParam());
}

/////////////////////////////////////////////////
TEST_F(FactoryTest, ActorPlugin)
{
  // Load a world with the default physics engine
  this->Load("worlds/empty.world", true);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::string skinFile("walk.dae");
  std::string animFile("walk.dae");
  std::string animName("walking");
  std::ostringstream actorStr;
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <skin>"
    << "    <filename>" << skinFile << "</filename>"
    << "  </skin>"
    << "  <animation name='" << animName << "'>"
    << "    <filename>" << animFile << "</filename>"
    << "    <interpolate_x>true</interpolate_x>"
    << "  </animation>"
    << "  <plugin name='actorPlugin' filename='libActorPlugin.so'>"
    << "    <target>0 -5 1.2138</target>"
    << "    <target_weight>1.15</target_weight>"
    << "    <obstacle_weight>1.8</obstacle_weight>"
    << "    <animation_factor>5.1</animation_factor>"
    << "  </plugin>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  this->WaitUntilEntitySpawn(actorName, 300, 10);

  auto model = this->GetModel(actorName);
  ASSERT_NE(nullptr, model);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_NE(nullptr, actor);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_NE(nullptr, sdf);

  // Check the plugin is still there
  EXPECT_TRUE(sdf->HasElement("plugin"));
  EXPECT_EQ(1u, actor->GetPluginCount());

  // Check the plugin was loaded and set a custom trajectory
  EXPECT_NE(nullptr, actor->CustomTrajectory());
}

/////////////////////////////////////////////////
void FactoryTest::ActorAll(const std::string &_physicsEngine)
{
  this->Load("worlds/empty.world", true, _physicsEngine);

  // Check there is no actor yet
  std::string actorName("test_actor");
  EXPECT_FALSE(this->GetModel(actorName));

  // Spawn from actor SDF string
  std::string skinFile("moonwalk.dae");
  std::string animFile("walk.dae");
  std::string animName("walk_animation");
  std::ostringstream actorStr;
  actorStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name ='" << actorName << "'>"
    << "  <skin>"
    << "    <filename>" << skinFile << "</filename>"
    << "  </skin>"
    << "  <animation name='" << animName << "'>"
    << "    <filename>" << animFile << "</filename>"
    << "  </animation>"
    << "  <script>"
    << "    <trajectory id='0' type='" << animName << "'>"
    << "      <waypoint>"
    << "        <time>0.0</time>"
    << "        <pose>1 1 0 0 0 0</pose>"
    << "      </waypoint>"
    << "      <waypoint>"
    << "        <time>1.0</time>"
    << "        <pose>2 2 0 0 0 0</pose>"
    << "      </waypoint>"
    << "    </trajectory>"
    << "  </script>"
    << "</actor>"
    << "</sdf>";

  msgs::Factory msg;
  msg.set_sdf(actorStr.str());
  this->factoryPub->Publish(msg);

  // Wait until actor was spawned
  this->WaitUntilEntitySpawn(actorName, 300, 10);

  auto model = this->GetModel(actorName);
  ASSERT_TRUE(model != nullptr);

  // Convert to actor
  auto actor = boost::dynamic_pointer_cast<physics::Actor>(model);
  ASSERT_TRUE(actor != nullptr);

  // Check it is active
  EXPECT_TRUE(actor->IsActive());

  // Check the SDF
  auto sdf = actor->GetSDF();
  ASSERT_TRUE(sdf != nullptr);

  // Check skin is still there
  EXPECT_TRUE(sdf->HasElement("skin"));
  auto skinElem = sdf->GetElement("skin");
  ASSERT_TRUE(skinElem != nullptr);
  EXPECT_EQ(skinElem->GetElement("filename")->Get<std::string>(),
      skinFile);

  // Check the animation is still there
  EXPECT_TRUE(sdf->HasElement("animation"));
  auto animationElem = sdf->GetElement("animation");
  ASSERT_TRUE(animationElem != nullptr);
  EXPECT_EQ(animationElem->GetElement("filename")->Get<std::string>(),
      animFile);

  // Check the script is still there
  EXPECT_TRUE(sdf->HasElement("script"));

  // Check links were added
  EXPECT_TRUE(sdf->HasElement("link"));

  auto linkElem = sdf->GetElement("link");
  EXPECT_TRUE(linkElem != nullptr);

  int linkCount = 0;
  while (linkElem)
  {
    linkCount++;
    linkElem = linkElem->GetNextElement("link");
  }
  EXPECT_EQ(linkCount, 32);

  // Check the anim animation was loaded
  auto skelAnims = actor->SkeletonAnimations();
  EXPECT_FALSE(skelAnims.empty());
  EXPECT_EQ(skelAnims.size(), 1u);
  EXPECT_TRUE(skelAnims[animName] != nullptr);

  // Check no custom trajectory has been set
  EXPECT_EQ(nullptr, actor->CustomTrajectory());
}

/////////////////////////////////////////////////
TEST_P(FactoryTest, ActorAll)
{
  ActorAll(GetParam());
}

/////////////////////////////////////////////////
void FactoryTest::Clone(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  ignition::math::Pose3d testPose;
  this->Load("worlds/pr2.world", true, _physicsEngine);

  // clone the pr2
  std::string name = "pr2";
  msgs::Factory msg;
  ignition::math::Pose3d clonePose;
  clonePose.Set(ignition::math::Vector3d(2, 3, 0.5),
      ignition::math::Quaterniond::Identity);
  msgs::Set(msg.mutable_pose(), clonePose);
  msg.set_clone_model_name(name);
  this->factoryPub->Publish(msg);

  // Wait for the pr2 clone to spawn
  std::string cloneName = name + "_clone";
  this->WaitUntilEntitySpawn(cloneName, 100, 100);

  EXPECT_TRUE(this->HasEntity(cloneName));
  testPose = EntityPose(cloneName);
  EXPECT_TRUE(testPose.Pos().Equal(clonePose.Pos(), 0.1));

  // Verify properties of the pr2 clone with the original model.
  // Check model
  physics::ModelPtr model = this->GetModel(name);
  ASSERT_TRUE(model != NULL);
  physics::ModelPtr modelClone = this->GetModel(cloneName);
  ASSERT_TRUE(modelClone != NULL);
  EXPECT_EQ(model->GetJointCount(), modelClone->GetJointCount());
  EXPECT_EQ(model->GetLinks().size(), modelClone->GetLinks().size());
  EXPECT_EQ(model->GetSensorCount(), modelClone->GetSensorCount());
  EXPECT_EQ(model->GetPluginCount(), modelClone->GetPluginCount());
  EXPECT_EQ(model->GetAutoDisable(), modelClone->GetAutoDisable());

  // Check links
  physics::Link_V links = model->GetLinks();
  physics::Link_V linkClones = modelClone->GetLinks();
  for (unsigned int i = 0; i < links.size(); ++i)
  {
    physics::LinkPtr link = links[i];
    physics::LinkPtr linkClone = linkClones[i];

    EXPECT_EQ(link->GetSensorCount(), linkClone->GetSensorCount());
    EXPECT_EQ(link->GetKinematic(), linkClone->GetKinematic());

    // Check collisions
    physics::Collision_V collisions = link->GetCollisions();
    physics::Collision_V collisionClones = linkClone->GetCollisions();
    EXPECT_EQ(collisions.size(), collisionClones.size());
    for (unsigned int j = 0; j < collisions.size(); ++j)
    {
      physics::CollisionPtr collision = collisions[j];
      physics::CollisionPtr collisionClone = collisionClones[j];
      EXPECT_EQ(collision->GetShapeType(), collisionClone->GetShapeType());
      EXPECT_EQ(collision->GetMaxContacts(), collisionClone->GetMaxContacts());

      // Check surface
      physics::SurfaceParamsPtr surface = collision->GetSurface();
      physics::SurfaceParamsPtr cloneSurface = collisionClone->GetSurface();
      EXPECT_EQ(surface->collideWithoutContact,
          cloneSurface->collideWithoutContact);
      EXPECT_EQ(surface->collideWithoutContactBitmask,
          cloneSurface->collideWithoutContactBitmask);
    }

    // Check inertial
    physics::InertialPtr inertial = link->GetInertial();
    physics::InertialPtr inertialClone = linkClone->GetInertial();
    EXPECT_EQ(inertial->Mass(), inertialClone->Mass());
    EXPECT_EQ(inertial->CoG(), inertialClone->CoG());
    // Expect Inertial MOI to match, even if Principal moments
    // and inertial frames change
    if (_physicsEngine != "bullet")
    {
      EXPECT_EQ(inertial->Ign().MOI(), inertialClone->Ign().MOI());
    }
    else
    {
      // the default == tolerance of 1e-6, is too strict for bullet
      EXPECT_TRUE(inertial->Ign().MOI().Equal(
             inertialClone->Ign().MOI(), 1e-5));
    }
  }

  // Check joints
  physics::Joint_V joints = model->GetJoints();
  physics::Joint_V jointClones = modelClone->GetJoints();
  for (unsigned int i = 0; i < joints.size(); ++i)
  {
    physics::JointPtr joint = joints[i];
    physics::JointPtr jointClone = jointClones[i];
    EXPECT_EQ(joint->DOF(), jointClone->DOF());
    for (unsigned j = 0; j < joint->DOF(); ++j)
    {
      EXPECT_NEAR(joint->UpperLimit(j), jointClone->UpperLimit(j), g_tolerance);
      EXPECT_NEAR(joint->LowerLimit(j), jointClone->LowerLimit(j), g_tolerance);
      EXPECT_EQ(joint->GetEffortLimit(j), jointClone->GetEffortLimit(j));
      EXPECT_EQ(joint->GetVelocityLimit(j), jointClone->GetVelocityLimit(j));
      EXPECT_EQ(joint->GetStopStiffness(j), jointClone->GetStopStiffness(j));
      EXPECT_EQ(joint->GetStopDissipation(j),
          jointClone->GetStopDissipation(j));
      EXPECT_EQ(joint->LocalAxis(j), jointClone->LocalAxis(j));
      EXPECT_EQ(joint->GetDamping(j), jointClone->GetDamping(j));
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
//  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
//      rendering::RenderEngine::NONE)
//    return;
//
//  math::Pose setPose, testPose;
//  Load("worlds/empty.world");
//  setPose.Set(ignition::math::Vector3d(-5, 0, 5),
//              ignition::math::Quaterniond(0, IGN_DTOR(15), 0));
//  SpawnCamera("camera_model", "camera_sensor2", setPose.pos,
//      setPose.rot.GetAsEuler());
//
//  unsigned char *img = NULL;
//  unsigned int width;
//  unsigned int height;
//  GetFrame("camera_sensor2", &img, width, height);
//  ASSERT_EQ(width, static_cast<unsigned int>(320));
//  ASSERT_EQ(height, static_cast<unsigned int>(240));
//
//  unsigned int diffMax = 0;
//  unsigned int diffSum = 0;
//  double diffAvg = 0;
//  ImageCompare(&img, &empty_world_camera1,
//      width, height, 3, diffMax, diffSum, diffAvg);
//  // PrintImage("empty_world_camera1", &img, width, height, 3);
//  ASSERT_LT(diffSum, static_cast<unsigned int>(100));
//  ASSERT_EQ(static_cast<unsigned int>(0), diffMax);
//  ASSERT_EQ(0.0, diffAvg);
// }


/////////////////////////////////////////////////
TEST_F(LightFactoryTest, SpawnLight)
{
  Load("worlds/empty.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // create lights using ~/factory/light topic
  this->lightFactoryPub = this->node->Advertise<msgs::Light>("~/factory/light");
  std::string light1Name = "new1_light";
  std::string light2Name = "new2_light";

  msgs::Light msg;
  msg.set_name(light1Name);
  ignition::math::Pose3d light1Pose = ignition::math::Pose3d(0, 2, 1, 0, 0, 0);
  msgs::Set(msg.mutable_pose(), light1Pose);
  this->lightFactoryPub->Publish(msg);

  msg.set_name(light2Name);
  ignition::math::Pose3d light2Pose = ignition::math::Pose3d(1, 0, 5, 0, 2, 0);
  msgs::Set(msg.mutable_pose(), light2Pose);
  this->lightFactoryPub->Publish(msg);

  // wait for light to spawn
  int sleep = 0;
  int maxSleep = 50;
  while ((!world->LightByName(light1Name) || !world->LightByName(light2Name)) &&
      sleep++ < maxSleep)
  {
    common::Time::MSleep(100);
  }

  // verify lights in world
  physics::LightPtr light1 = world->LightByName(light1Name);
  physics::LightPtr light2 = world->LightByName(light2Name);
  ASSERT_NE(nullptr, light1);
  ASSERT_NE(nullptr, light2);

  EXPECT_EQ(light1Name, light1->GetScopedName());
  EXPECT_EQ(light1Pose, light1->WorldPose());

  EXPECT_EQ(light2Name, light2->GetScopedName());
  EXPECT_EQ(light2Pose, light2->WorldPose());
}

/////////////////////////////////////////////////
void LightFactoryTest::ResponseCb(ConstResponsePtr &_msg)
{
  if (_msg->request() == "entity_info")
  {
    std::lock_guard<std::mutex> lk(this->resMutex);
    this->responseCbCalled = true;
    this->resMsg = *_msg;
  }
}

/////////////////////////////////////////////////
void LightFactoryTest::ModelInfoCb(ConstModelPtr &_msg)
{
  if (_msg->name() == "test_model")
  {
    std::lock_guard<std::mutex> lk(this->modMutex);
    this->modelInfoCbCalled = true;
    this->modMsg = *_msg;
  }
}

/////////////////////////////////////////////////
TEST_F(LightFactoryTest, SpawnModelWithLight)
{
  Load("worlds/empty.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Create a publisher to request for entity information.
  this->requestPub
    = this->node->Advertise<msgs::Request>("~/request");
  msgs::Request reqMsg;
  reqMsg.set_request("entity_info");
  reqMsg.set_data("test_model");

  // Create a subscriber to get a response for entity information.
  this->responseSub
    = this->node->Subscribe<msgs::Response>("~/response",
    &LightFactoryTest::ResponseCb, dynamic_cast<LightFactoryTest*>(this));

  // Create a subscriber to get a model info.
  this->modelInfoSub
    = this->node->Subscribe<msgs::Model>("~/model/info",
    &LightFactoryTest::ModelInfoCb, dynamic_cast<LightFactoryTest*>(this));

  // Create a publisher to spawn a model.
  this->factoryPub
    = this->node->Advertise<msgs::Factory>("~/factory");
  msgs::Factory facMsg;

  // Request for entity informaiton.
  reqMsg.set_id(0);
  requestPub->Publish(reqMsg);

  // Wait for a reponse.
  common::Time::MSleep(1000);

  // Expect there is nothing yet.
  {
    std::lock_guard<std::mutex> lk(this->resMutex);
    EXPECT_TRUE(this->responseCbCalled) << "No response callback";
    EXPECT_EQ(0, this->resMsg.id()) << "Wrong message ID";
    EXPECT_EQ("nonexistent", this->resMsg.response());
    this->responseCbCalled = false;
  }

  // Spawn a model which has a light under its link.
  std::ostringstream modelStr;
  modelStr << "<sdf version='" << SDF_VERSION << "'>"
           << "  <model name='test_model'>"
           << "    <link name='link'>"
           << "      <light name='spawned_light' type='point'>"
           << "        <pose>0 0 0 0 0 0</pose>"
           << "        <attenuation>"
           << "          <range>0.20</range>"
           << "          <linear>0.10</linear>"
           << "        </attenuation>"
           << "        <diffuse>1 1 1 1</diffuse>"
           << "        <specular>1 1 1 1</specular>"
           << "      </light>"
           << "      <visual name='marker'>"
           << "        <pose>0 0 0 0 0 0</pose>"
           << "        <geometry>"
           << "          <sphere>"
           << "            <radius>0.025</radius>"
           << "          </sphere>"
           << "        </geometry>"
           << "        <material>"
           << "          <ambient>1 1 1 1</ambient>"
           << "          <diffuse>1 1 1 1</diffuse>"
           << "          <specular>1 1 1 1</specular>"
           << "          <emissive>1 1 1 1</emissive>"
           << "        </material>"
           << "      </visual>"
           << "    </link>"
           << "  </model>"
           << "</sdf>";

  facMsg.set_sdf(modelStr.str());
  this->factoryPub->Publish(facMsg);

  // Wait for it to be spawned.
  this->WaitUntilEntitySpawn("test_model", 300, 10);

  // Request for entity information.
  reqMsg.set_id(1);
  requestPub->Publish(reqMsg);

  // Wait for a reponse.
  common::Time::MSleep(1000);

  // Verify the light exists in the physics engine.
  {
    std::lock_guard<std::mutex> lk(this->resMutex);
    EXPECT_TRUE(this->responseCbCalled) << "No response callback";
    EXPECT_EQ(1, this->resMsg.id()) << "Wrong message ID";
    EXPECT_NE("nonexistent", this->resMsg.response())
      << "The model does not exist.";
    msgs::Model modelMsg;
    modelMsg.ParseFromString(this->resMsg.serialized_data());
    EXPECT_EQ(1, modelMsg.link_size())
      << "The number of links must be 1.";
    msgs::Link linkMsg = modelMsg.link(0);
    EXPECT_EQ(1, linkMsg.light_size())
      << "The number of lights must be 1.";
    msgs::Light lightMsg = linkMsg.light(0);
    EXPECT_EQ("test_model::link::spawned_light", lightMsg.name())
      << "The name of the light is wrong.";
  }

  // Verify the correct model info has been published for the rendering engine.
  {
    std::lock_guard<std::mutex> lk(this->modMutex);
    EXPECT_TRUE(this->modelInfoCbCalled) << "No model info callback";
    EXPECT_EQ(1, this->modMsg.link_size())
      << "The number of links must be 1.";
    msgs::Link linkMsg = this->modMsg.link(0);
    EXPECT_EQ(1, linkMsg.light_size())
      << "The number of lights must be 1.";
    msgs::Light lightMsg = linkMsg.light(0);
    EXPECT_EQ("test_model::link::spawned_light", lightMsg.name())
      << "The name of the light is wrong.";
  }
}

//////////////////////////////////////////////////
TEST_F(FactoryTest, FilenameModelDatabase)
{
  this->Load("worlds/empty.world", true);

  // Test database
  common::SystemPaths::Instance()->AddModelPaths(
    PROJECT_SOURCE_PATH "/test/models/testdb");

  // World
  auto world = physics::get_world("default");
  ASSERT_NE(nullptr, world);

  // Publish factory msg
  msgs::Factory msg;
  msg.set_sdf_filename("model://cococan");

  auto pub = this->node->Advertise<msgs::Factory>("~/factory");
  pub->Publish(msg);

  // Wait for it to be spawned
  int sleep = 0;
  int maxSleep = 50;
  while (!world->ModelByName("cococan") && sleep++ < maxSleep)
  {
    common::Time::MSleep(100);
  }

  // Check model was spawned
  ASSERT_NE(nullptr, world->ModelByName("cococan"));
}

//////////////////////////////////////////////////
#ifdef HAVE_IGNITION_FUEL_TOOLS
TEST_F(FactoryTest, FilenameFuelURL)
{
  this->Load("worlds/empty.world", true);

  // World
  auto world = physics::get_world("default");
  ASSERT_NE(nullptr, world);

  msgs::Factory msg;
  msg.set_sdf_filename(
      "https://api.ignitionfuel.org/1.0/chapulina/models/Test box");

  auto pub = this->node->Advertise<msgs::Factory>("~/factory");
  pub->Publish(msg);

  // Wait for it to be spawned
  int sleep = 0;
  int maxSleep = 50;
  while (!world->ModelByName("test_box") && sleep++ < maxSleep)
  {
    common::Time::MSleep(100);
  }

  // Check model was spawned
  ASSERT_NE(nullptr, world->ModelByName("test_box"));
}
#endif

//////////////////////////////////////////////////
TEST_P(FactoryTest, InvalidMeshInsertion)
{
  std::string physicsEngine = GetParam();
  this->Load("worlds/empty.world", true, physicsEngine);

  auto world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  std::string newModelName("new_model");
  std::string trimeshPath = "file://does-not-exist.stl";

  // Current world state
  physics::WorldState worldState(world);

  // Try to insert a model with a geometry which references a URI
  // which doesn't exist. The server should at least not crash, though it
  // may vary between physics engines whether the model is actually added.
  // It should print some errors, but it may or may not have addded the
  // (partly) faulty model.
  std::stringstream newModelStr;
  newModelStr << "<sdf version='" << sdf::SDF::Version() << "'>"
               << "<model name ='" << newModelName << "'>"
               << "<link name ='link'>"
               << "  <collision name ='collision'>"
               << "    <geometry>"
               << "      <mesh>"
               << "        <uri>" << trimeshPath << "</uri>"
               << "        <scale>1 1 1</scale>"
               << "      </mesh>"
               << "    </geometry>"
               << "  </collision>"
               << "  <visual name ='visual'>"
               << "    <geometry>"
               << "      <mesh>"
               << "        <uri>" << trimeshPath << "</uri>"
               << "        <scale>1 1 1</scale>"
               << "      </mesh>"
               << "    </geometry>"
               << "  </visual>"
               << "</link>"
               << "</model>"
               << "</sdf>";

  // Set state which includes insertion
  std::vector<std::string> insertions;
  insertions.push_back(newModelStr.str());
  worldState.SetInsertions(insertions);
  world->SetState(worldState);
  world->Step(1);
}

//////////////////////////////////////////////////
TEST_P(FactoryTest, InvalidMeshInsertionWithWorld)
{
  std::string physicsEngine = GetParam();
  this->Load("test/worlds/invalid_mesh_uri.world", true, physicsEngine);
  auto world = physics::get_world("mesh_test_world");
  ASSERT_TRUE(world != NULL);
  world->Step(1);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, FactoryTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
