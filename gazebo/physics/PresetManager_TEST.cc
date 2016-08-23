/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"
#include "gazebo/physics/PresetManager.hh"
#include "sdf/sdf.hh"

using namespace gazebo;

class PresetManagerTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, GetSetPresetParam)
{
  physics::Preset preset("preset1");
  EXPECT_EQ(preset.Name(), "preset1");
  std::string foo = "foo";
  boost::any value1 = foo;
  EXPECT_FALSE(preset.HasParam("key1"));
  EXPECT_TRUE(preset.SetParam("key1", value1));
  EXPECT_FALSE(preset.SetParam("", value1));
  EXPECT_TRUE(preset.HasParam("key1"));
  EXPECT_FALSE(preset.HasParam(""));
  boost::any value2;
  EXPECT_FALSE(preset.GetParam("", value2));
  EXPECT_FALSE(preset.GetParam("key_does_not_exist", value2));
  EXPECT_TRUE(preset.GetParam("key1", value2));
  try
  {
    EXPECT_EQ(boost::any_cast<std::string>(value1),
        boost::any_cast<std::string>(value2));
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Bad any cast in PresetManager_TEST" << std::endl;
    FAIL();
  }
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, GetSetProfileParam)
{
  EXPECT_NO_THROW(physics::PresetManager(NULL, NULL));

  // Load preset test world
  Load("test/worlds/presets.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  physics::PresetManagerPtr presetManager = world->GetPresetManager();

  EXPECT_FALSE(presetManager->CurrentProfile(""));
  EXPECT_FALSE(presetManager->CurrentProfile("preset_does_not_exist"));
  EXPECT_EQ(presetManager->CurrentProfile(), "preset_1");

  // current
  double max_step_size = 0.9;
  EXPECT_FALSE(presetManager->SetCurrentProfileParam("param_does_not_exist",
      max_step_size));
  EXPECT_TRUE(presetManager->SetCurrentProfileParam("max_step_size",
      0.8));
  EXPECT_TRUE(presetManager->SetProfileParam("preset_1", "max_step_size",
      max_step_size));

  boost::any value2;
  EXPECT_FALSE(presetManager->GetCurrentProfileParam("param_does_not_exist",
      value2));
  EXPECT_TRUE(presetManager->GetCurrentProfileParam("max_step_size", value2));
  EXPECT_TRUE(presetManager->GetProfileParam("preset_1", "max_step_size",
      value2));

  try
  {
    EXPECT_DOUBLE_EQ(max_step_size, boost::any_cast<double>(value2));
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Bad any cast in PresetManager_TEST" << std::endl;
    FAIL();
  }

  // Trying to set a preset profile that does not exist should return false.
  EXPECT_FALSE(presetManager->SetProfileParam("preset_does_not_exist",
      "max_step_size", 10));
  EXPECT_FALSE(presetManager->SetProfileParam("preset_2",
      "param_does_not_exist", 10));

  EXPECT_TRUE(presetManager->SetProfileParam("preset_2", "max_step_size", 10));

  // Trying to get from a nonexistent preset profile should return false.
  EXPECT_FALSE(presetManager->GetProfileParam("preset_does_not_exist",
      "max_step_size", value2));
  // Trying to get a nonexistent param should return false.
  EXPECT_FALSE(presetManager->GetProfileParam("preset_2",
      "param_does_not_exist", value2));

  EXPECT_TRUE(presetManager->GetProfileParam("preset_2", "max_step_size",
      value2));
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, CreateRemoveProfile)
{
  // Load preset test world
  Load("test/worlds/presets.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  physics::PresetManagerPtr presetManager = world->GetPresetManager();
  boost::any value;

  presetManager->RemoveProfile("preset_2");
  EXPECT_FALSE(presetManager->HasProfile("preset_2"));

  // Remove the current profile
  presetManager->RemoveProfile("preset_1");
  // Test setting and getting when there is no current profile
  EXPECT_EQ(presetManager->CurrentProfile(), "");
  // Even with valid keys, setting/getting from the current profile will fail
  EXPECT_FALSE(presetManager->SetCurrentProfileParam("max_step_size", 1));
  EXPECT_FALSE(presetManager->GetCurrentProfileParam("max_step_size", value));

  // Even with valid keys, setting/getting from the removed profile will fail
  EXPECT_FALSE(presetManager->SetProfileParam("preset_1", "max_step_size", 1));
  EXPECT_FALSE(presetManager->GetProfileParam("preset_1", "max_step_size",
      value));

  EXPECT_FALSE(presetManager->HasProfile("preset_1"));

  // Create a profile using (name)
  EXPECT_FALSE(presetManager->CreateProfile(""));
  EXPECT_TRUE(presetManager->CreateProfile("preset_1"));
  EXPECT_TRUE(presetManager->SetProfileParam("preset_1", "max_step_size", 1.0));
  EXPECT_TRUE(presetManager->GetProfileParam("preset_1", "max_step_size",
      value));

  try
  {
    EXPECT_DOUBLE_EQ(1.0, boost::any_cast<double>(value));
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Bad any cast in PresetManager_TEST" << std::endl;
    FAIL();
  }
}

/////////////////////////////////////////////////
TEST_F(PresetManagerTest, SDF)
{
  // Load preset test world
  Load("test/worlds/presets.world", true);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  physics::PresetManagerPtr presetManager = world->GetPresetManager();

  EXPECT_TRUE(!presetManager->ProfileSDF("profile_does_not_exist"));

  {
    // Try to set using a bad SDF element
    sdf::SDF worldSDF;
    sdf::ElementPtr physicsSDF;

    worldSDF.SetFromString(
        "<sdf version = \"1.5\">\
          <world name = \"default\">\
            <physics name = \"preset_3\" type = \"ode\">\
              <max_step_size>0.03</max_step_size>\
              <bad_sdf_tag/>\
            </physics>\
          </world>\
        </sdf>");
    EXPECT_EQ(presetManager->CreateProfile(physicsSDF), "");
  }

  {
    sdf::SDF worldSDF;
    sdf::ElementPtr physicsSDF;
    boost::any value;

    worldSDF.SetFromString(
      "<sdf version = \"1.5\">\
        <world name = \"default\">\
          <physics name = \"preset_3\" type = \"ode\">\
            <max_step_size>0.03</max_step_size>\
            <magnetic_field>0 0 0</magnetic_field>\
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
    physicsSDF = worldSDF.Root()->GetElement("world")->GetElement("physics");
    EXPECT_EQ(presetManager->CreateProfile(physicsSDF), "preset_3");
    EXPECT_TRUE(presetManager->HasProfile("preset_3"));

    // Compare the SDF as strings
    EXPECT_EQ(presetManager->ProfileSDF("preset_3")->ToString(""),
        physicsSDF->ToString(""));
    EXPECT_TRUE(presetManager->ProfileSDF("preset_3") != NULL);
    EXPECT_TRUE(presetManager->CurrentProfile("preset_3"));
    try
    {
      EXPECT_TRUE(presetManager->GetCurrentProfileParam("max_step_size",
          value));
      EXPECT_DOUBLE_EQ(boost::any_cast<double>(value), 0.03);
      EXPECT_TRUE(presetManager->GetCurrentProfileParam("min_step_size",
          value));
      EXPECT_DOUBLE_EQ(boost::any_cast<double>(value), 0.003);
    }
    catch(boost::bad_any_cast &_e)
    {
      gzerr << "Bad any cast in PresetManager_TEST" << std::endl;
      FAIL();
    }

    // GenerateSDFFromPreset
    sdf::ElementPtr generatedPhysicsSDF = NULL;
    presetManager->GenerateSDFFromPreset("this_preset_does_not_exist",
        generatedPhysicsSDF);
    // Call doesn't do anything
    ASSERT_TRUE(generatedPhysicsSDF == NULL);

    presetManager->GenerateSDFFromPreset("preset_3", generatedPhysicsSDF);
    // Compare the SDF as strings
    ASSERT_TRUE(generatedPhysicsSDF != NULL);

    EXPECT_EQ(generatedPhysicsSDF->ToString(""), physicsSDF->ToString(""));
  }

  {
    // Try to set a null pointer
    sdf::ElementPtr nullElement = NULL;
    presetManager->ProfileSDF("preset_3", nullElement);
    // Should have no effect
    EXPECT_TRUE(presetManager->ProfileSDF("preset_3") != NULL);
  }

  {
    // Try to set a non-physics element
    sdf::SDF worldSDF;
    sdf::ElementPtr sceneSDF;

    worldSDF.SetFromString(
      "<sdf version = \"1.5\">\
        <world name = \"default\">\
          <scene>\
            <ambient>0.1 0.1 0.1 1</ambient>\
            <background>1 1 1 1</background>\
            <shadows>false</shadows>\
            <grid>false</grid>\
          </scene>\
        </world>\
      </sdf>");
    sceneSDF = worldSDF.Root()->GetElement("world")->GetElement("scene");
    EXPECT_EQ(presetManager->CreateProfile(sceneSDF), "");
    EXPECT_FALSE(presetManager->ProfileSDF("default_physics", sceneSDF));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
