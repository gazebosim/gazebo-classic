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

#include <gtest/gtest.h>
#include <gazebo/rendering/rendering.hh>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class Issue346Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test for issue #346
TEST_F(Issue346Test, SaveLights)
{
  Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  std::string spotLightName = "spot_light";
  math::Vector3 spotLightPos(1, 2, 5);
  math::Vector3 spotLightRot(0, 0, 0.5);

  std::string pointLightName = "point_light";
  math::Vector3 pointLightPos(4, 3, 8);
  math::Vector3 pointLightRot(0, 0.8, 0.1);

  // Spawn two lights: one spot light and one point light
  SpawnLight(spotLightName, "spot", spotLightPos, spotLightRot);
  SpawnLight(pointLightName, "point", pointLightPos, pointLightRot);

  boost::filesystem::path pathOut(boost::filesystem::current_path());
  boost::filesystem::create_directories(pathOut /
      boost::filesystem::path("tmp"));
  std::string filenameOut = pathOut.string() +
      "/tmp/346_save_lights.world";

  // Save lights to a world file
  world->Save(filenameOut);

  // Load the saved world file
  sdf::SDFPtr sdf(new sdf::SDF);
  ASSERT_TRUE(sdf::init(sdf));
  ASSERT_TRUE(sdf::readFile(common::find_file(filenameOut), sdf));
  ASSERT_TRUE(sdf->Root() != NULL);

  // Verify there is one spot light and one point light
  int hasSpotLight = 0;
  int hasPointLight = 0;
  sdf::ElementPtr worldElem = sdf->Root()->GetElement("world");
  ASSERT_TRUE(worldElem != NULL);
  sdf::ElementPtr lightElem = worldElem->GetElement("light");
  while (lightElem)
  {
    std::string name = lightElem->Get<std::string>("name");
    math::Pose pose = lightElem->Get<math::Pose>("pose");
    if (name == spotLightName)
    {
      hasSpotLight++;
      EXPECT_TRUE(pose.pos == spotLightPos);
      EXPECT_TRUE(pose.rot == spotLightRot);
    }
    else if (name == pointLightName)
    {
      hasPointLight++;
      EXPECT_TRUE(pose.pos == pointLightPos);
      EXPECT_TRUE(pose.rot == pointLightRot);
    }
    lightElem = lightElem->GetNextElement("light");
  }

  EXPECT_EQ(hasSpotLight, 1);
  EXPECT_EQ(hasPointLight, 1);

  // Remove temp directory
  boost::filesystem::remove_all(pathOut.string() + "/tmp");
}
