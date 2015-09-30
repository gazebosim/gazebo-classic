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

