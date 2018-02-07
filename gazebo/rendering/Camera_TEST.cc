/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class Camera_TEST: public RenderingFixture
{
};


/////////////////////////////////////////////////
TEST_F(Camera_TEST, Create)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);
  ASSERT_TRUE(scene != nullptr);

  // test creating rgb camera
  {
    rendering::CameraPtr camera =
        scene->CreateCamera("test_camera_rgb8", false);
    ASSERT_TRUE(camera != nullptr);

    unsigned int width = 320;
    unsigned int height = 240;
    std::string format = "R8G8B8";
    double hfov = 0.78;
    double near = 0.01;
    double far = 100.0;
    std::stringstream ss;
    ss << "<sdf version='" << SDF_VERSION << "'>"
       << "  <camera>"
       << "    <horizontal_fov>" << hfov << "</horizontal_fov>"
       << "    <image>"
       << "      <width>" << width << "</width>"
       << "      <height>" << height << "</height>"
       << "      <format>" << format << "</format>"
       << "    </image>"
       << "    <clip>"
       << "      <near>" << near << "</near>" << "<far>" << far << "</far>"
       << "    </clip>"
       << "  </camera>"
       << "</sdf>";
    sdf::ElementPtr cameraSDF(new sdf::Element);
    sdf::initFile("camera.sdf", cameraSDF);
    sdf::readString(ss.str(), cameraSDF);
    camera->Load(cameraSDF);
    camera->Init();

    EXPECT_EQ(width, camera->ImageWidth());
    EXPECT_EQ(height, camera->ImageHeight());
    unsigned int channels = 3u;
    unsigned int bytesPerChannel = 1u;
    unsigned int bytesPerPixel = channels * bytesPerChannel;
    unsigned int imageMemSize = width * height * bytesPerPixel;
    EXPECT_EQ(bytesPerPixel, camera->ImageDepth());
    EXPECT_EQ(imageMemSize, camera->ImageMemorySize());
    EXPECT_EQ(format, camera->ImageFormat());
    EXPECT_DOUBLE_EQ(hfov, camera->HFOV().Radian());
    EXPECT_NEAR(near, camera->NearClip(), 1e-3);
    EXPECT_DOUBLE_EQ(far, camera->FarClip());

    scene->RemoveCamera(camera->Name());
  }

  // test creating grayscale camera
  {
    rendering::CameraPtr camera = scene->CreateCamera("test_camera_l8", false);
    ASSERT_TRUE(camera != nullptr);

    unsigned int width = 640;
    unsigned int height = 480;
    std::string format = "L8";
    double hfov = 0.5;
    double near = 0.05;
    double far = 20.0;
    std::stringstream ss;
    ss << "<sdf version='" << SDF_VERSION << "'>"
       << "  <camera>"
       << "    <horizontal_fov>" << hfov << "</horizontal_fov>"
       << "    <image>"
       << "      <width>" << width << "</width>"
       << "      <height>" << height << "</height>"
       << "      <format>" << format << "</format>"
       << "    </image>"
       << "    <clip>"
       << "      <near>" << near << "</near>" << "<far>" << far << "</far>"
       << "    </clip>"
       << "  </camera>"
       << "</sdf>";
    sdf::ElementPtr cameraSDF(new sdf::Element);
    sdf::initFile("camera.sdf", cameraSDF);
    sdf::readString(ss.str(), cameraSDF);
    camera->Load(cameraSDF);
    camera->Init();

    EXPECT_EQ(width, camera->ImageWidth());
    EXPECT_EQ(height, camera->ImageHeight());
    unsigned int channels = 1u;
    unsigned int bytesPerChannel = 1u;
    unsigned int bytesPerPixel = channels * bytesPerChannel;
    unsigned int imageMemSize = width * height * bytesPerPixel;
    EXPECT_EQ(bytesPerPixel, camera->ImageDepth());
    EXPECT_EQ(imageMemSize, camera->ImageMemorySize());
    EXPECT_EQ(format, camera->ImageFormat());
    EXPECT_DOUBLE_EQ(hfov, camera->HFOV().Radian());
    EXPECT_NEAR(near, camera->NearClip(), 1e-3);
    EXPECT_DOUBLE_EQ(far, camera->FarClip());

    scene->RemoveCamera(camera->Name());
  }


  // test creating 16 bit rgb camera
  {
    rendering::CameraPtr camera =
        scene->CreateCamera("test_camera_rgb16", false);
    ASSERT_TRUE(camera != nullptr);

    unsigned int width = 500;
    unsigned int height = 300;
    std::string format = "R16G16B16";
    double hfov = 1.05;
    double near = 0.001;
    double far = 200.0;
    std::stringstream ss;
    ss << "<sdf version='" << SDF_VERSION << "'>"
       << "  <camera>"
       << "    <horizontal_fov>" << hfov << "</horizontal_fov>"
       << "    <image>"
       << "      <width>" << width << "</width>"
       << "      <height>" << height << "</height>"
       << "      <format>" << format << "</format>"
       << "    </image>"
       << "    <clip>"
       << "      <near>" << near << "</near>" << "<far>" << far << "</far>"
       << "    </clip>"
       << "  </camera>"
       << "</sdf>";
    sdf::ElementPtr cameraSDF(new sdf::Element);
    sdf::initFile("camera.sdf", cameraSDF);
    sdf::readString(ss.str(), cameraSDF);
    camera->Load(cameraSDF);
    camera->Init();

    EXPECT_EQ(width, camera->ImageWidth());
    EXPECT_EQ(height, camera->ImageHeight());
    unsigned int channels = 3u;
    unsigned int bytesPerChannel = 2u;
    unsigned int bytesPerPixel = channels * bytesPerChannel;
    unsigned int imageMemSize = width * height * bytesPerPixel;
    EXPECT_EQ(bytesPerPixel, camera->ImageDepth());
    EXPECT_EQ(imageMemSize, camera->ImageMemorySize());
    EXPECT_EQ(format, camera->ImageFormat());
    EXPECT_DOUBLE_EQ(hfov, camera->HFOV().Radian());
    EXPECT_NEAR(near, camera->NearClip(), 1e-3);
    EXPECT_DOUBLE_EQ(far, camera->FarClip());

    scene->RemoveCamera(camera->Name());
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
