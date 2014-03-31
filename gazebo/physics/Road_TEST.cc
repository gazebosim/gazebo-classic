/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/Road.hh"
#include "test/util.hh"

using namespace gazebo;

class RoadTest : public gazebo::testing::AutoLogFixture { };

TEST_F(RoadTest, Texture)
{
  std::ostringstream roadStr;
  roadStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<road name='my_road'>"
    << "<width>5</width>"
    << "<point>0 0 0</point>"
    << "<point>25 0 0</point>"
    << "<texture>primary</texture>"
    << "</road>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr roadSDF(new sdf::SDF);
  roadSDF->SetFromString(roadStr.str());

  physics::RoadPtr road(
      new physics::Road(physics::BasePtr()));
  sdf::ElementPtr elem = roadSDF->root;
  ASSERT_TRUE(elem);
  elem = elem->GetElement("road");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("lanes_2");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("motorway");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("lanes_6");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("trunk");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("lanes_4");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("secondary");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("residential");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("lane_1");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("footway");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("sidewalk");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("tertiary");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("");
  ASSERT_TRUE(elem);
  road->Load(elem);

  elem->GetElement("texture")->Set("Unknow_Value");
  ASSERT_TRUE(elem);
  road->Load(elem);

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
