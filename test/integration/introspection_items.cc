/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#include "gazebo/util/IntrospectionClient.hh"

using namespace gazebo;

class IntrospectionItemsTest : public ServerFixture,
     public testing::WithParamInterface<const char*>
{
  /// \brief Check registered introspection items.
  /// \param[in] _physicsEngine Physics engine type.
  public: void RegisteredItems(const std::string &_physicsEngine);

  /// \brief Check available introspection items for a model.
  /// \param[in] _model Pointer to the model.
  private: void ModelItems(physics::ModelPtr _model);

  /// \brief Check available introspection items for a link.
  /// \param[in] _link Pointer to the link.
  private: void LinkItems(physics::LinkPtr _link);

  /// \brief Check available introspection items for a joint.
  /// \param[in] _joint Pointer to the joint.
  private: void JointItems(physics::JointPtr _joint);

  /// \brief Stores the list of items expected to be registered.
  private: std::vector<std::string> itemsExpected;
};

/////////////////////////////////////////////////
void IntrospectionItemsTest::RegisteredItems(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
    return;

  this->Load("test/worlds/deeply_nested_models.world", true, _physicsEngine);
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  auto physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // Create an introspection client
  gazebo::util::IntrospectionClient client;

  // Wait for the managers to come online
  auto managerIds = client.WaitForManagers(std::chrono::seconds(2));
  EXPECT_FALSE(managerIds.empty());

  // Pick up the first manager.
  auto id = *managerIds.begin();
  EXPECT_FALSE(id.empty());

  // Get available items
  std::set<std::string> itemsAvailable;
  EXPECT_TRUE(client.Items(id, itemsAvailable));
  EXPECT_FALSE(itemsAvailable.empty());

  // Check items expected for the world
  std::vector<std::string> queries;
  queries.push_back("time/sim_time");

  auto worldUri = world->URI();
  for (auto &query : queries)
  {
    common::URI uri(worldUri);
    uri.Query().Insert("p", query);
    EXPECT_FALSE(itemsAvailable.find(uri.Str()) == itemsAvailable.end());
  }

  // Get list of items expected for models and their children
  auto models = world->GetModels();
  EXPECT_FALSE(models.empty());

  for (auto model : models)
    this->ModelItems(model);

  // Check all items are registered
  for (auto &item : this->itemsExpected)
  {
    gzdbg << "Checking [" << item << "] is registered" << std::endl;
    EXPECT_FALSE(itemsAvailable.find(item) == itemsAvailable.end());
  }

  // Remove models
  for (auto model : models)
    world->RemoveModel(model);
  EXPECT_TRUE(world->GetModels().empty());

  // Expect items not to be registered
  itemsAvailable.clear();
  EXPECT_TRUE(client.Items(id, itemsAvailable));
  for (auto &item : this->itemsExpected)
  {
    gzdbg << "Checking [" << item << "] is not registered" << std::endl;
    EXPECT_TRUE(itemsAvailable.find(item) == itemsAvailable.end());
  }
}

/////////////////////////////////////////////////
void IntrospectionItemsTest::ModelItems(physics::ModelPtr _model)
{
  auto links = _model->GetLinks();
  for (auto link : links)
    this->LinkItems(link);

  auto nestedModels = _model->NestedModels();
  for (auto nestedModel : nestedModels)
    this->ModelItems(nestedModel);

  auto joints = _model->GetJoints();
  for (auto joint : joints)
    this->JointItems(joint);

  std::vector<std::string> queries;
  queries.push_back("pose3d/world_pose");
  queries.push_back("vector3d/world_linear_velocity");
  queries.push_back("vector3d/world_angular_velocity");
  queries.push_back("vector3d/world_linear_acceleration");
  queries.push_back("vector3d/world_angular_acceleration");

  auto modelUri = _model->URI();
  for (auto &query : queries)
  {
    common::URI uri(modelUri);
    uri.Query().Insert("p", query);
    this->itemsExpected.push_back(uri.Str());
  }
}

/////////////////////////////////////////////////
void IntrospectionItemsTest::LinkItems(physics::LinkPtr _link)
{
  auto linkUri = _link->URI();

  std::vector<std::string> queries;
  queries.push_back("pose3d/world_pose");
  queries.push_back("vector3d/world_linear_velocity");
  queries.push_back("vector3d/world_angular_velocity");
  queries.push_back("vector3d/world_linear_acceleration");
  queries.push_back("vector3d/world_angular_acceleration");

  for (auto &query : queries)
  {
    common::URI uri(linkUri);
    uri.Query().Insert("p", query);
    this->itemsExpected.push_back(uri.Str());
  }
}

/////////////////////////////////////////////////
void IntrospectionItemsTest::JointItems(physics::JointPtr _joint)
{
  auto jointUri = _joint->URI();

  for (size_t i = 0; i < _joint->GetAngleCount(); ++i)
  {
    std::vector<std::string> queries;
    queries.push_back("axis/" + std::to_string(i) + "/double/position");
    queries.push_back("axis/" + std::to_string(i) + "/double/velocity");

    for (auto &query : queries)
    {
      common::URI uri(jointUri);
      uri.Query().Insert("p", query);
      this->itemsExpected.push_back(uri.Str());
    }
  }
}

/////////////////////////////////////////////////
TEST_P(IntrospectionItemsTest, RegisteredItems)
{
  RegisteredItems(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, IntrospectionItemsTest,
    PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
