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

#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/LightState.hh"

using namespace gazebo;

class LightStateTest : public ServerFixture { };

//////////////////////////////////////////////////
TEST_F(LightStateTest, SDFConstructor)
{
  // Create the state sdf
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<light name='light_0'>"
    << "  <pose>0 0 0.5 0 0 0</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("light"));

  // Create the light state
  physics::LightState lightState(stateElem->GetElement("light"));

  // Check light state against values from the sdf string
  EXPECT_EQ(lightState.Pose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
}

//////////////////////////////////////////////////
TEST_F(LightStateTest, LightConstructor)
{
  ignition::math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);

  // Load a world
  this->Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");

  // Create a base to be the light's parent
  physics::BasePtr basePtr;
  basePtr.reset(new physics::Base(physics::BasePtr()));
  basePtr->SetName("world_root_element");
  basePtr->SetWorld(world);

  // Create the light message
  msgs::Light lightMsg;
  lightMsg.set_name("test_light");
  msgs::Set(lightMsg.mutable_pose(), pose);

  // Create the light
  physics::LightPtr lightPtr(new physics::Light(basePtr));
  EXPECT_TRUE(lightPtr != NULL);

  lightPtr->ProcessMsg(lightMsg);
  EXPECT_EQ(lightPtr->GetWorldPose(), pose);

  // Create the light state
  physics::LightState lightState(lightPtr, common::Time(1), common::Time(1), 1);

  // Check light state against values from the message
  EXPECT_EQ(lightState.Pose(), pose);
}

//////////////////////////////////////////////////
TEST_F(LightStateTest, FillSDF)
{
  // Create the state sdf
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<light name='light_0'>"
    << "  <pose>1 2 3 0.1 0.2 0.3</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("light"));

  sdf::ElementPtr lightElem = stateElem->GetElement("light");

  // Create the light state
  physics::LightState lightState(lightElem);

  // Fill SDF
  sdf::ElementPtr filledSDF(new sdf::Element);
  sdf::initFile("light_state.sdf", filledSDF);
  lightState.FillSDF(filledSDF);

  EXPECT_TRUE(filledSDF->HasAttribute("name"));
  EXPECT_EQ(filledSDF->GetName(), lightElem->GetName());
  EXPECT_TRUE(filledSDF->HasElement("pose"));
  EXPECT_EQ(filledSDF->GetElement("pose")->Get<ignition::math::Pose3d>(),
      lightElem->GetElement("pose")->Get<ignition::math::Pose3d>());
}

//////////////////////////////////////////////////
TEST_F(LightStateTest, Operators)
{
  // Create the state sdf
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<light name='light_0'>"
    << "  <pose>1 2 3 0 0 0</pose>"
    << "</light>"
    << "<light name='light_1'>"
    << "  <pose>4 5 6 0 0 0</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("light"));

  sdf::ElementPtr light0 = stateElem->GetElement("light");
  sdf::ElementPtr light1 = light0->GetNextElement("light");

  // Create light states
  physics::LightState lightState0(light0);
  physics::LightState lightState1(light1);

  // Check states are different
  EXPECT_NE(lightState0.GetName(), lightState1.GetName());
  EXPECT_NE(lightState0.Pose(), lightState1.Pose());
  EXPECT_FALSE((lightState0 - lightState1).IsZero());

  // Check subtraction
  auto lightState2 = lightState0 - lightState1;
  EXPECT_EQ(lightState2.Pose(), ignition::math::Pose3d(-3, -3, -3, 0, 0, 0));

  lightState2 = lightState1 - lightState0;
  EXPECT_EQ(lightState2.Pose(), ignition::math::Pose3d(3, 3, 3, 0, 0, 0));

  // Check addition
  lightState2 = lightState1 + lightState0;
  EXPECT_EQ(lightState2.Pose(), ignition::math::Pose3d(5, 7, 9, 0, 0, 0));

  // Copy by assignment
  lightState1 = lightState0;

  // Check states are equal
  EXPECT_EQ(lightState0.GetName(), lightState1.GetName());
  EXPECT_EQ(lightState0.Pose(), lightState1.Pose());
  EXPECT_TRUE((lightState0 - lightState1).IsZero());
}

//////////////////////////////////////////////////
TEST_F(LightStateTest, Print)
{
  // Create the state sdf
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<light name='light_0'>"
    << "  <pose>1 2 3 0.1 0.2 0.3</pose>"
    << "</light>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);

  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));

  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));

  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("light"));

  // Create the light state
  physics::LightState lightState(stateElem->GetElement("light"));

  // Stream
  std::ostringstream strOut;
  strOut << lightState;

  EXPECT_STREQ(strOut.str().c_str(),
      "<light name='light_0'><pose>1.000 2.000 3.000 0.100 0.200 0.300 "\
      "</pose></light>");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

