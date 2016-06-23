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
#include <map>
#include <string>
#include <utility>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

const double g_tolerance = 1e-6;

class CollideTest : public ServerFixture,
                    public testing::WithParamInterface<const char *>
{
  /// \brief Test collision checking between spheres.
  /// \param[in] _physicsEngine Physics engine to use.
  public: void Spheres(const std::string &_physicsEngine);
};

void OnContacts(ConstContactsPtr &/*_msg*/)
{
}

typedef std::map<double, std::pair<physics::ModelPtr, physics::ModelPtr>>
        mapSeparatedModels;
/////////////////////////////////////////////////
// Collide spheres:
// Load world with many pairs of spheres in contact.
// Disable physics and verify collision checking.
void CollideTest::Spheres(const std::string &_physicsEngine)
{
  // Load collide_spheres world
  Load("worlds/collide_spheres.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("collide_spheres");
  ASSERT_NE(world, nullptr);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_NE(physics, nullptr);
  ASSERT_EQ(physics->GetType(), _physicsEngine);

  // Disable physics updates
  world->EnablePhysicsEngine(false);

  // Models with 1mm and 100mm radius
  auto models = world->GetModels();
  mapSeparatedModels mmRadius, dmRadius;
  for (const auto model : models)
  {
    const auto name = model->GetName();
    if (name.compare(0, 2, "mm") == 0)
    {
      double separationRatio = std::stod(name.substr(2, 2)) / 10;
      if (name.at(4) == 'A')
      {
        mmRadius[separationRatio].first = model;
      }
      else if (name.at(4) == 'B')
      {
        mmRadius[separationRatio].second = model;
      }
      else
      {
        gzerr << "Unrecognized model name: " << name << std::endl;
      }
    }
    else if (name.compare(0, 2, "dm") == 0)
    {
      double separationRatio = std::stod(name.substr(2, 2)) / 10;
      if (name.at(4) == 'A')
      {
        dmRadius[separationRatio].first = model;
      }
      else if (name.at(4) == 'B')
      {
        dmRadius[separationRatio].second = model;
      }
      else
      {
        gzerr << "Unrecognized model name: " << name << std::endl;
      }
    }
  }

  EXPECT_EQ(mmRadius.size(), 12u);
  EXPECT_EQ(dmRadius.size(), 12u);

  // Confirm no models are missing a partner
  for (const auto mmPair : mmRadius)
  {
    gzdbg << "Checking mm radius pair with separation ratio "
          << mmPair.first
          << std::endl;
    ASSERT_NE(mmPair.second.first, nullptr);
    ASSERT_NE(mmPair.second.second, nullptr);

    // compute distance between object centers
    auto modelA = mmPair.second.first;
    auto modelB = mmPair.second.second;
    auto positionDiff = modelA->GetWorldPose().Ign().Pos()
                      - modelB->GetWorldPose().Ign().Pos();
    EXPECT_DOUBLE_EQ(positionDiff.Length(), mmPair.first * 1e-3);
  }
  for (const auto dmPair : dmRadius)
  {
    gzdbg << "Checking dm radius pair with separation ratio "
          << dmPair.first
          << std::endl;
    ASSERT_NE(dmPair.second.first, nullptr);
    ASSERT_NE(dmPair.second.second, nullptr);

    // compute distance between object centers
    auto modelA = dmPair.second.first;
    auto modelB = dmPair.second.second;
    auto positionDiff = modelA->GetWorldPose().Ign().Pos()
                      - modelB->GetWorldPose().Ign().Pos();
    EXPECT_DOUBLE_EQ(positionDiff.Length(), dmPair.first * 1e-1);
  }

  // You have to subscribe to a contacts topic in order to use
  // the C++ API, otherwise it skips it to save CPU time
  auto contactSub = this->node->Subscribe("~/physics/contacts", &OnContacts);

  world->Step(1);

  // Contact data
  auto contactManager = physics->GetContactManager();
  ASSERT_NE(contactManager, nullptr);
  unsigned int contactCount = contactManager->GetContactCount();
  EXPECT_GE(contactCount, 14u);
  auto contacts = contactManager->GetContacts();

  for (unsigned int i = 0; i < contactCount; ++i)
  {
    const auto contact = contacts[i];
    EXPECT_EQ(contact->count, 1);
    if (contact->count != 1)
      continue;

    gzdbg << contact->collision1->GetModel()->GetScopedName()
          << " "
          << contact->collision2->GetModel()->GetScopedName()
          << " "
          << contact->normals[0] << " normal, "
          << contact->depths[0] << " depth"
          << std::endl;

    std::string name1(contact->collision1->GetModel()->GetScopedName());
    std::string name2(contact->collision2->GetModel()->GetScopedName());
    EXPECT_EQ(0, name1.compare(0, 4, name2.substr(0, 4)));
    double radius;
    if (name1.compare(0, 2, "mm") == 0)
    {
      radius = 1e-3;
    }
    else if (name1.compare(0, 2, "dm") == 0)
    {
      radius = 1e-1;
    }
    else
    {
      ADD_FAILURE() << " unrecognized model name prefix: "
                    << name1.substr(0, 2);
      continue;
    }
    double separation = radius * std::stod(name1.substr(2, 2)) / 10;
    double depth = 2*radius - separation;
    EXPECT_NEAR(depth, contact->depths[0], g_tolerance);

    // normal is undefined when sphere centers coincide
    if (separation != 0.0)
    {
      // otherwise expect unit vectors parallel to UnitX
      // {1, 0, 0} or {-1, 0, 0}
      auto normal = contact->normals[0].Ign();
      EXPECT_NEAR(normal.Length(), 1.0, g_tolerance);
      EXPECT_NEAR(0.0,
        ignition::math::Vector3d::UnitX.Cross(normal).Length(),
        g_tolerance);
    }
  }
}

/////////////////////////////////////////////////
TEST_P(CollideTest, Spheres)
{
  Spheres(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, CollideTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
