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
#include <string>
#include <stdlib.h>

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/rendering/Distortion.hh"
#include "gazebo/rendering/Camera.hh"

#include "test/util.hh"


using namespace gazebo;

class Distortion_TEST : public RenderingFixture { };

sdf::ElementPtr CreateDistortionSDFElement(double _k1, double _k2,
    double _k3, double _p1, double _p2, double _cx, double _cy)
{
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='distorted_cam'>"
    << "  <static>true</static>"
    << "  <pose>0 0 0 0 0 0</pose>"
    << "  <link name ='body'>"
    << "    <sensor name ='distorted_cam' type ='camera'>"
    << "      <always_on>1</always_on>"
    << "      <update_rate>30</update_rate>"
    << "      <visualize>true</visualize>"
    << "      <camera>"
    << "        <horizontal_fov>0.78539816339744828</horizontal_fov>"
    << "        <image>"
    << "          <width>100</width>"
    << "          <height>100</height>"
    << "          <format>R8G8B8</format>"
    << "        </image>"
    << "        <clip>"
    << "          <near>0.1</near><far>100</far>"
    << "        </clip>"
    << "        <distortion>"
    << "          <k1>" << _k1 << "</k1>"
    << "          <k2>" << _k2 << "</k2>"
    << "          <k3>" << _k3 << "</k3>"
    << "          <p1>" << _p1 << "</p1>"
    << "          <p2>" << _p2 << "</p2>"
    << "          <center>" << _cx << " " << _cy << "</center>"
    << "        </distortion>"
    << "      </camera>"
    << "    </sensor>"
    << "  </link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr cameraSDF(new sdf::SDF);
  cameraSDF->SetFromString(newModelStr.str());

  sdf::ElementPtr elem = cameraSDF->Root()
    ->GetElement("model")
    ->GetElement("link")
    ->GetElement("sensor")
    ->GetElement("camera")
    ->GetElement("distortion");
  return elem;
}

/////////////////////////////////////////////////
TEST_F(Distortion_TEST, DistortionModelBasics)
{
  Load("worlds/empty.world");

  rendering::Distortion distortion;

  // check the default crop behavior of the different distortion models
  // undistorted
  distortion.Load(CreateDistortionSDFElement(0, 0, 0, 0, 0, 0.5, 0.5));
  EXPECT_EQ(distortion.Crop(), false);

  // barrel distortion
  distortion.Load(
    CreateDistortionSDFElement(-0.1, -0.05, -0.01, 0, 0, 0.5, 0.5));
  EXPECT_EQ(distortion.Crop(), true);

  // pincushion distortion
  distortion.Load(CreateDistortionSDFElement(0.1, 0.05, 0.01, 0, 0, 0.5, 0.5));
  EXPECT_EQ(distortion.Crop(), false);

  // getters and setters
  distortion.SetCrop(false);
  EXPECT_EQ(distortion.Crop(), false);
  distortion.SetCrop(true);
  EXPECT_EQ(distortion.Crop(), true);
  EXPECT_DOUBLE_EQ(distortion.K1(), 0.1);
  EXPECT_DOUBLE_EQ(distortion.K2(), 0.05);
  EXPECT_DOUBLE_EQ(distortion.K3(), 0.01);
  EXPECT_DOUBLE_EQ(distortion.P1(), 0.0);
  EXPECT_DOUBLE_EQ(distortion.P2(), 0.0);
  EXPECT_DOUBLE_EQ(distortion.Center().X(), 0.5);
  EXPECT_DOUBLE_EQ(distortion.Center().Y(), 0.5);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
