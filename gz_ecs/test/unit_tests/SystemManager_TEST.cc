/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/ecs_core/EntityManager.hh"
#include "gazebo/ecs_core/System.hh"
#include "gazebo/ecs_core/SystemManager.hh"


class DummySystem : public gazebo::ecs_core::System
{
  public:
    static bool WasLoaded;
    static bool WasUpdated;
    static int NumResults;

    virtual void Init(gazebo::ecs_core::EntityQuery &_query)
    {
      DummySystem::WasLoaded = true;
    }

    virtual void Update(
        double _dt, const gazebo::ecs_core::EntityQueryResult &_result)
    {
      DummySystem::WasUpdated = true;
      DummySystem::NumResults = _result.NumResults();
    }
};

bool DummySystem::WasLoaded = false;
bool DummySystem::WasUpdated = false;
int DummySystem::NumResults = 0;

// Component Types for testing
struct TC1
{
  float itemOne;
};


/////////////////////////////////////////////////
TEST(SystemManager, InitialEntityManagerIsNull)
{
  gazebo::ecs_core::SystemManager uut;
  EXPECT_EQ(nullptr, uut.GetEntityManager());
}

/////////////////////////////////////////////////
TEST(SystemManager, SetEntityManager)
{
  gazebo::ecs_core::SystemManager uut;
  gazebo::ecs_core::EntityManager em;
  uut.SetEntityManager(&em);
  EXPECT_EQ(&em, uut.GetEntityManager());
}

/////////////////////////////////////////////////
TEST(SystemManager, CanLoadSystem)
{
  gazebo::ecs_core::SystemManager uut;
  gazebo::ecs_core::EntityManager em;
  uut.SetEntityManager(&em);

  DummySystem::WasLoaded = false;
  uut.LoadSystem<DummySystem>();

  EXPECT_TRUE(DummySystem::WasLoaded);
}

/////////////////////////////////////////////////
TEST(SystemManager, CanUpdateSystem)
{
  gazebo::ecs_core::SystemManager uut;
  gazebo::ecs_core::EntityManager em;
  uut.SetEntityManager(&em);
  uut.LoadSystem<DummySystem>();

  DummySystem::WasUpdated = false;
  uut.Update(123);
  EXPECT_TRUE(DummySystem::WasUpdated);
}

/////////////////////////////////////////////////
TEST(SystemManager, CanGetEntities)
{
  gazebo::ecs_core::SystemManager uut;
  gazebo::ecs_core::EntityManager em;
  gazebo::ecs_core::Entity entity = em.CreateEntity();
  em.AddComponent<TC1>(entity);
  uut.SetEntityManager(&em);
  uut.LoadSystem<DummySystem>();

  DummySystem::NumResults = 0;
  em.Update();
  uut.Update(123);
  EXPECT_EQ(1, DummySystem::NumResults);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

