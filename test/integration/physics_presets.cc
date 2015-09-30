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

#include <boost/any.hpp>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class PresetManagerTest : public ServerFixture,
    public testing::WithParamInterface<const char*>
{
};

/////////////////////////////////////////////////
TEST_P(PresetManagerTest, InitializeAllPhysicsEngines)
{
  const std::string physicsEngineName = GetParam();
  Load("test/worlds/presets.world", false, physicsEngineName);
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  double maxStepSize;
  EXPECT_TRUE(physicsEngine->Param<double>("max_step_size", maxStepSize));
  EXPECT_FLOAT_EQ(maxStepSize, 0.01);
  if (physicsEngineName == "ode" || physicsEngineName == "bullet")
  {
    double value;
    EXPECT_TRUE(physicsEngine->Param<double>("min_step_size", value));
    EXPECT_FLOAT_EQ(value, 0.001);

    EXPECT_TRUE(physicsEngine->Param<double>("cfm", value));
    EXPECT_FLOAT_EQ(value, 0.01);

    EXPECT_TRUE(physicsEngine->Param<double>("erp", value));
    EXPECT_FLOAT_EQ(value, 0.3);


    EXPECT_TRUE(physicsEngine->Param<double>("contact_surface_layer", value));
    EXPECT_FLOAT_EQ(value, 0.002);

    EXPECT_TRUE(physicsEngine->Param<double>("sor", value));
    EXPECT_FLOAT_EQ(value, 1.4);

    EXPECT_TRUE(physicsEngine->Param<double>("iters", value));
    EXPECT_EQ(value, 50);
  }
  if (physicsEngineName == "ode")
  {
    bool ratio = false;
    EXPECT_TRUE(physicsEngine->Param<bool>("inertia_ratio_reduction", ratio));
    EXPECT_TRUE(ratio);

    double vel;
    EXPECT_TRUE(physicsEngine->Param<double>(
          "contact_max_correcting_vel", vel));
    EXPECT_FLOAT_EQ(vel, 200);
  }
  if (physicsEngineName == "bullet")
  {
    bool impulse = true;
    EXPECT_TRUE(physicsEngine->Param<bool>("split_impulse", impulse));
    EXPECT_FALSE(impulse);
  }
  if (physicsEngineName == "simbody")
  {
    double value;
    EXPECT_TRUE(physicsEngine->Param<double>("accuracy", value));

    EXPECT_FLOAT_EQ(value, 0.01);

    EXPECT_TRUE(physicsEngine->Param<double>("max_transient_velocity", value));
    EXPECT_FLOAT_EQ(value, 0.001);
  }
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, MultipleDefaults)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }
  EXPECT_EQ(presetManager->CurrentProfile(), "preset_1");
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, NoDefault)
{
  Load("test/worlds/presets_nodefault.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }
  EXPECT_EQ(presetManager->CurrentProfile(), "preset_1");
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, SetProfileParam)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  EXPECT_TRUE(presetManager->SetCurrentProfileParam("max_step_size", 10.0));

  double value;
  EXPECT_TRUE(physicsEngine->Param("max_step_size", value));
  EXPECT_FLOAT_EQ(value, 10.0);

  // preset_2 is not the current profile, so we do not expect to see a change
  // in the physics engine when we change preset_2.
  EXPECT_TRUE(presetManager->SetProfileParam("preset_2", "max_step_size",
        20));
  EXPECT_TRUE(physicsEngine->Param("max_step_size", value));
  EXPECT_FLOAT_EQ(value, 10.0);
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, SetCurrentProfile)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();

  if (!presetManager)
  {
    FAIL();
  }

  std::vector<std::string> profileNames(presetManager->AllProfiles());
  EXPECT_EQ(profileNames.size(), 3u);

  presetManager->CurrentProfile("preset_2");

  EXPECT_EQ(presetManager->CurrentProfile(), "preset_2");

  try
  {
    double dblValue;
    EXPECT_TRUE(physicsEngine->Param<double>("max_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.02);

    EXPECT_TRUE(physicsEngine->Param<double>("min_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.002);

    int intValue;
    EXPECT_TRUE(physicsEngine->Param<int>("iters", intValue));
    EXPECT_EQ(intValue, 100);

    EXPECT_TRUE(physicsEngine->Param<double>("cfm", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.02);

    EXPECT_TRUE(physicsEngine->Param<double>("erp", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.6);
  }
  catch(const boost::bad_any_cast& e)
  {
    FAIL();
  }
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, CreateProfileFromSDF)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  sdf::SDF worldSDF;
  worldSDF.SetFromString(
      "<sdf version = \"1.5\">\
        <world name = \"default\">\
          <physics name = \"preset_3\" type = \"ode\">\
            <max_step_size>0.03</max_step_size>\
            <ode>\
              <solver>\
                <min_step_size>0.003</min_step_size>\
                <iters>150</iters>\
                <sor>1.6</sor>\
              </solver>\
              <constraints>\
                <cfm>0.03</cfm>\
                <erp>0.7</erp>\
              </constraints>\
            </ode>\
          </physics>\
        </world>\
      </sdf>");
  sdf::ElementPtr physicsSDF = worldSDF.Root()->GetElement("world")
      ->GetElement("physics");
  presetManager->CreateProfile(physicsSDF);
  presetManager->CurrentProfile("preset_3");
  try
  {
    double dblValue;
    int intValue;

    EXPECT_TRUE(physicsEngine->Param<double>("max_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.03);

    EXPECT_TRUE(physicsEngine->Param<double>("min_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.003);

    EXPECT_TRUE(physicsEngine->Param<int>("iters", intValue));
    EXPECT_EQ(intValue, 150);

    EXPECT_TRUE(physicsEngine->Param<double>("cfm", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.03);

    EXPECT_TRUE(physicsEngine->Param<double>("erp", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.7);
  }
  catch(const boost::bad_any_cast& e)
  {
    FAIL();
  }
}

TEST_F(PresetManagerTest, BackwardsCompatibilityTest)
{
  Load("worlds/empty.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }
  try
  {
    double dblValue;
    int intValue;
    bool boolValue = true;

    EXPECT_TRUE(physicsEngine->Param<double>("max_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.001);

    EXPECT_TRUE(physicsEngine->Param<int>("iters", intValue));
    EXPECT_EQ(intValue, 50);

    EXPECT_TRUE(physicsEngine->Param<double>("cfm", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.0);

    EXPECT_TRUE(physicsEngine->Param<double>("erp", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.2);

    EXPECT_TRUE(physicsEngine->Param<double>("contact_max_correcting_vel",
          dblValue));
    EXPECT_FLOAT_EQ(dblValue, 100);

    EXPECT_TRUE(physicsEngine->Param<double>("contact_surface_layer",
          dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.001);

    EXPECT_TRUE(physicsEngine->Param<double>("sor", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 1.3);

    EXPECT_TRUE(physicsEngine->Param<double>("min_step_size", dblValue));
    EXPECT_FLOAT_EQ(dblValue, 0.0001);

    EXPECT_TRUE(physicsEngine->Param<bool>("inertia_ratio_reduction",
          boolValue));
    EXPECT_FALSE(boolValue);
  }
  catch(const boost::bad_any_cast& e)
  {
    FAIL();
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PresetManagerTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
