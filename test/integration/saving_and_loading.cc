/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <ignition/math/Vector3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

//////////////////////////////////////////////////
class SavingLoadingTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  /// \brief Test scaling a model and then saving it. The scale should show up
  /// in the state SDF instead of the initial model SDF.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void SaveScaledModel(const std::string &_physicsEngine);

  /// \brief Test loading a world with models scaled in the world state.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void LoadScaledModels(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void SavingLoadingTest::SaveScaledModel(const std::string &_physicsEngine)
{
  // load a world with simple shapes
  this->Load("worlds/shapes.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  auto physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  // Get box and check scale
  auto box = world->GetModel("box");
  ASSERT_TRUE(box != NULL);
  EXPECT_EQ(box->Scale(), ignition::math::Vector3d::One);

  // Create transport
  auto node = transport::NodePtr(new transport::Node());
  node->Init();
  auto modelPub = node->Advertise<msgs::Model>("~/model/modify");

  // scale box
  ignition::math::Vector3d newScale(0.1, 2, 3);
  msgs::Model msg;
  msg.set_name("box");

  msgs::Set(msg.mutable_scale(), newScale);
  modelPub->Publish(msg);

  // Check box has been scaled
  int sleep = 0;
  int maxSleep = 10;
  while (box->Scale() != newScale && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(100);
    sleep++;
  }
  EXPECT_EQ(box->Scale(), newScale);

  // request SDF to save world
  auto response = transport::request("default", "world_sdf_save");
  EXPECT_NE(response->response(), "error");

  std::string str = response->serialized_data();
  // Remove strange characters from the front
  str = str.substr(3);

  sdf::SDF sdf_parsed;
  sdf_parsed.SetFromString(str);
  ASSERT_TRUE(sdf_parsed.Root()->HasElement("world"));

  auto worldElem = sdf_parsed.Root()->GetElement("world");

  // Check all model sizes (scales)
  EXPECT_TRUE(worldElem->HasElement("model"));
  auto modelElem = worldElem->GetElement("model");
  while (modelElem)
  {
    EXPECT_TRUE(modelElem->HasElement("link"));
    EXPECT_TRUE(modelElem->GetElement("link")->HasElement("visual"));
    EXPECT_TRUE(modelElem->GetElement("link")->HasElement("collision"));
    EXPECT_TRUE(modelElem->GetElement("link")->GetElement("visual")->
        HasElement("geometry"));
    EXPECT_TRUE(modelElem->GetElement("link")->GetElement("collision")->
        HasElement("geometry"));

    auto visualGeom = modelElem->GetElement("link")->GetElement("visual")->
        GetElement("geometry");
    auto collisionGeom = modelElem->GetElement("link")->
        GetElement("collision")->GetElement("geometry");

    EXPECT_TRUE(modelElem->HasAttribute("name"));
    auto name = modelElem->GetAttribute("name")->GetAsString();

    // Check that all sizes are still one
    if (name == "box")
    {
      gzdbg << "Checking box model geometry size." << std::endl;
      EXPECT_TRUE(visualGeom->HasElement("box"));
      EXPECT_TRUE(collisionGeom->HasElement("box"));
      EXPECT_TRUE(visualGeom->GetElement("box")->HasElement("size"));
      EXPECT_TRUE(collisionGeom->GetElement("box")->HasElement("size"));

      auto visualSize = visualGeom->GetElement("box")->
          Get<ignition::math::Vector3d>("size");
      auto collisionSize = collisionGeom->GetElement("box")->
          Get<ignition::math::Vector3d>("size");

      EXPECT_EQ(visualSize, ignition::math::Vector3d::One);
      EXPECT_EQ(collisionSize, ignition::math::Vector3d::One);
    }
    else if (name == "sphere")
    {
      gzdbg << "Checking sphere model geometry size." << std::endl;
      EXPECT_TRUE(visualGeom->HasElement("sphere"));
      EXPECT_TRUE(collisionGeom->HasElement("sphere"));
      EXPECT_TRUE(visualGeom->GetElement("sphere")->HasElement("radius"));
      EXPECT_TRUE(collisionGeom->GetElement("sphere")->HasElement("radius"));

      auto visualSize = visualGeom->GetElement("sphere")->Get<double>("radius");
      auto collisionSize = collisionGeom->GetElement("sphere")->
          Get<double>("radius");

      EXPECT_EQ(visualSize, 0.5);
      EXPECT_EQ(collisionSize, 0.5);
    }
    else if (name == "cylinder")
    {
      gzdbg << "Checking cylinder model geometry size." << std::endl;
      EXPECT_TRUE(visualGeom->HasElement("cylinder"));
      EXPECT_TRUE(collisionGeom->HasElement("cylinder"));
      EXPECT_TRUE(visualGeom->GetElement("cylinder")->HasElement("radius"));
      EXPECT_TRUE(collisionGeom->GetElement("cylinder")->HasElement("radius"));
      EXPECT_TRUE(visualGeom->GetElement("cylinder")->HasElement("length"));
      EXPECT_TRUE(collisionGeom->GetElement("cylinder")->HasElement("length"));

      auto visualRadius = visualGeom->GetElement("cylinder")->
          Get<double>("radius");
      auto collisionRadius = collisionGeom->GetElement("cylinder")->
          Get<double>("radius");
      auto visualLength = visualGeom->GetElement("cylinder")->
          Get<double>("length");
      auto collisionLength = collisionGeom->GetElement("cylinder")->
          Get<double>("length");

      EXPECT_EQ(visualRadius, 0.5);
      EXPECT_EQ(collisionRadius, 0.5);
      EXPECT_EQ(visualLength, 1.0);
      EXPECT_EQ(collisionLength, 1.0);
    }

    modelElem = modelElem->GetNextElement("model");
  }

  // Check scale in all model states
  EXPECT_TRUE(worldElem->HasElement("state"));
  EXPECT_TRUE(worldElem->GetElement("state")->HasElement("model"));
  auto modelStateElem = worldElem->GetElement("state")->GetElement("model");
  while (modelStateElem)
  {
    EXPECT_TRUE(modelStateElem->HasAttribute("name"));
    auto name = modelStateElem->GetAttribute("name")->GetAsString();

    EXPECT_TRUE(modelStateElem->HasElement("scale"));
    auto scale = modelStateElem->Get<ignition::math::Vector3d>("scale");

    // Check that only box state has new scale
    if (name == "box")
    {
      gzdbg << "Checking box model state scale." << std::endl;
      EXPECT_EQ(scale, newScale);
    }
    else if (name == "sphere")
    {
      gzdbg << "Checking sphere model state scale." << std::endl;
      EXPECT_EQ(scale, ignition::math::Vector3d::One);
    }
    else if (name == "cylinder")
    {
      gzdbg << "Checking cylinder model state scale." << std::endl;
      EXPECT_EQ(scale, ignition::math::Vector3d::One);
    }

    modelStateElem = modelStateElem->GetNextElement("model");
  }
}

//////////////////////////////////////////////////
void SavingLoadingTest::LoadScaledModels(const std::string &_physicsEngine)
{
  // load a world which has models scaled in the state
  this->Load("test/worlds/scaled_shapes.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  auto physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);

  // Get box and check scale
  auto box = world->GetModel("box");
  ASSERT_TRUE(box != NULL);
  EXPECT_EQ(box->Scale(), ignition::math::Vector3d(3.34637, 2.87999, 0.123722));

  // Get sphere and check scale
  auto sphere = world->GetModel("sphere");
  ASSERT_TRUE(sphere != NULL);
  EXPECT_EQ(sphere->Scale(),
      ignition::math::Vector3d(0.402674, 0.402674, 0.402674));

  // Get cylinder and check scale
  auto cylinder = world->GetModel("cylinder");
  ASSERT_TRUE(cylinder != NULL);
  EXPECT_EQ(cylinder->Scale(),
      ignition::math::Vector3d(2.2969, 2.2969, 2.09564));
}

/////////////////////////////////////////////////
TEST_P(SavingLoadingTest, SaveScaledModel)
{
  this->SaveScaledModel(GetParam());
}

/////////////////////////////////////////////////
TEST_P(SavingLoadingTest, LoadScaledModels)
{
  this->LoadScaledModels(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, SavingLoadingTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
