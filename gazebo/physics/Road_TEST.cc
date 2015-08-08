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

#include "gazebo/physics/Road.hh"
#include "test/util.hh"

using namespace gazebo;

class RoadTest : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(RoadTest, Texture)
{
  std::ostringstream roadStr;
  roadStr << "<sdf version='" << SDF_VERSION << "'><road name='my_road'>"
    << "<width>5</width>"
    << "<point>0 0 0</point>"
    << "<point>25 0 0</point>"
    << "<material>"
    << "<script>"
    << "<name>primary</name>"
    << "</script>"
    << "</material>"
    << "</road></sdf>";

  sdf::ElementPtr roadSDF(new sdf::Element);
  sdf::initFile("road.sdf", roadSDF);
  sdf::readString(roadStr.str(), roadSDF);

  physics::RoadPtr road(
      new physics::Road(physics::BasePtr()));
  sdf::ElementPtr scriptElem = roadSDF->GetElement(
      "material")->GetElement("script");
  ASSERT_TRUE(scriptElem != NULL);
  road->Load(roadSDF);

  EXPECT_STREQ(road->GetName().c_str(), "my_road");
  EXPECT_STREQ(road->GetSDF()->GetElement("material")->
      GetElement("script")->Get<std::string>("name").c_str(), "primary");

  scriptElem->GetElement("name")->Set("lanes_2");
  road->Load(roadSDF);
  EXPECT_STREQ(road->GetSDF()->GetElement("material")->
      GetElement("script")->Get<std::string>("name").c_str(), "lanes_2");

  scriptElem->GetElement("name")->Set("motorway");
  road->Load(roadSDF);
  EXPECT_STREQ(road->GetSDF()->GetElement("material")->
      GetElement("script")->Get<std::string>("name").c_str(), "motorway");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
