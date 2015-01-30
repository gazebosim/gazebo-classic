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

#include "test/ServerFixture.hh"

#include <boost/any.hpp>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;

class PresetManagerTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, InitializeAllPhysicsEngines)
{
  {
    Load("test/worlds/presets.world", false, "ode");
    physics::WorldPtr world = physics::get_world("default");

    // check physics engine for all params for preset_1
    
    /*physics::ODEPhysicsPtr physicsEngine = dynamic_cast<physics::ODEPhysicsPtr>
      (world->GetPhysicsEngine());*/
    physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("max_step_size")), 0.01, 1e-4);
    EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 21);
    EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("iters")), 50);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("cfm")), 0.01, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("erp")), 0.03, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("contact_max_correcting_vel")), 200, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("contact_surface_layer")), 0.002, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("sor")), 1.4, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("min_step_size")), 0.001, 1e-4);
    EXPECT_TRUE(boost::any_cast<float>(physicsEngine->GetParam("inertia_ratio_reduction")));
  }

  {
    Load("test/worlds/presets.world", false, "simbody");
    physics::WorldPtr world = physics::get_world("default");
    /*physics::SimbodyPhysicsPtr physicsEngine = dynamic_cast<physics::SimbodyPhysicsPtr>
      (world->GetPhysicsEngine());*/
    physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("max_step_size")), 0.01, 1e-4);
    EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 21);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("accuracy")), 0.01, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("max_transient_velocity")), 0.01, 1e-4);
  }

  {
    Load("test/worlds/presets.world", false, "bullet");
    physics::WorldPtr world = physics::get_world("default");
    /*physics::BulletPhysicsPtr physicsEngine = dynamic_cast<physics::BulletPhysicsPtr>
      (world->GetPhysicsEngine());*/
    physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("max_step_size")), 0.01, 1e-4);
    EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 21);
    EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("iters")), 50);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("cfm")), 0.01, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("erp")), 0.03, 1e-4);
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("contact_surface_layer")), 0.002, 1e-4);
    //EXPECT_NEAR(physicsEngine->GetSORPGSW(), 1.4, 1e-4);  // ????

    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("sor")), 1.4, 1e-4);  // ????
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("min_step_size")), 0.001, 1e-4);
    EXPECT_FALSE(boost::any_cast<bool>(physicsEngine->GetParam("split_impulse")));
    EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("split_impulse_penetration_threshold")), -0.02, 1e-4);
  }
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, SetProfileParam)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");
  /*physics::ODEPhysicsPtr physicsEngine = dynamic_cast<physics::ODEPhysicsPtr>
    (world->GetPhysicsEngine());*/
  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManager *presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  EXPECT_TRUE(boost::any_cast<bool>(presetManager->SetCurrentProfileParam("max_contacts", 10)));
  EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 10);

  // preset_2 is not the current profile, so we do not expect to see a change.
  EXPECT_TRUE(presetManager->SetProfileParam("preset_2", "max_contacts", 50));
  EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 10);

  EXPECT_FALSE(presetManager->SetProfileParam("this_preset_does_not_exist", "max_contacts", 10));

  EXPECT_FALSE(presetManager->SetCurrentProfileParam("this_param_does_not_exist", 10));
  EXPECT_FALSE(presetManager->SetProfileParam("preset_2", "this_param_does_not_exist", 10));
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, SetCurrentProfile)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManager *presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  presetManager->SetCurrentProfile("preset_2");

  EXPECT_EQ(boost::any_cast<std::string>(presetManager->GetCurrentProfileName()), "preset_2");

  /*physics::ODEPhysicsPtr physicsEngine = dynamic_cast<physics::ODEPhysicsPtr>
    (world->GetPhysicsEngine());*/

  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("max_step_size")), 0.02, 1e-4);
  EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("max_contacts")), 42);
  EXPECT_EQ(boost::any_cast<int>(physicsEngine->GetParam("iters")), 100);
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("cfm")), 0.02, 1e-4);
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("erp")), 0.06, 1e-4);
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("contact_max_correcting_vel")), 400, 1e-4);
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("contact_surface_layer")), 0.004, 1e-4);
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("sor")), 1.5, 1e-4);  // ????
  EXPECT_NEAR(boost::any_cast<float>(physicsEngine->GetParam("min_step_size")), 0.002, 1e-4);
  EXPECT_FALSE(boost::any_cast<bool>(physicsEngine->GetParam("inertia_ratio_reduction")));
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, CreateProfileFromSDF)
{
  Load("test/worlds/presets.world", false, "ode");
  physics::WorldPtr world = physics::get_world("default");

  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  physics::PresetManager *presetManager = world->GetPresetManager();
  if (!presetManager)
  {
    FAIL();
  }

  // TODO

    /*"<physics name=\"preset_3\" type=\"ode\">\
      <max_step_size>0.003</max_step_size>\
      <max_contacts>64</max_contacts>\
      <ode>\
        <solver>\
          <min_step_size>0.003</min_step_size>\
          <iters>200</iters>\
          <sor>1.6</sor>\
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>\
        </solver>\
        <constraints>\
          <cfm>0.04</cfm>\
          <erp>0.7</erp>\
          <contact_max_correcting_vel>500</contact_max_correcting_vel>\
          <contact_surface_layer>0.008</contact_surface_layer>\
        </constraints>\
      </ode>\
    </physics>"*/

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
