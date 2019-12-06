/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "gazebo/common/Event.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class RenderEventsTest : public RenderingFixture
{
};

int g_callback1= 0;
int g_callback2 = 0;

std::string g_callbackData1;
std::string g_callbackData2;

/////////////////////////////////////////////////
void callback1(const std::string &_data)
{
  g_callback1++;
  g_callbackData1 = _data;
}

/////////////////////////////////////////////////
void callback2(const std::string &_data)
{
  g_callback2++;
  g_callbackData2 = _data;
}

/////////////////////////////////////////////////
TEST_F(RenderEventsTest, CameraRender)
{
  Load("worlds/empty.world");

  g_callback1 = 0;
  g_callback2 = 0;
  g_callbackData1 = "";
  g_callbackData2 = "";

  rendering::ScenePtr scene = gazebo::rendering::get_scene();
  rendering::CameraPtr camera = scene->CreateCamera("test_camera", false);

  // preRender connection
  event::ConnectionPtr cameraPreRenderConnection =
      rendering::Events::ConnectCameraPreRender(
      std::bind(&callback1, std::placeholders::_1));

  // postRender connection
  event::ConnectionPtr cameraPostRenderConnection =
      rendering::Events::ConnectCameraPostRender(
      std::bind(&callback2, std::placeholders::_1));

  // verify initial state
  EXPECT_EQ(0, g_callback1);
  EXPECT_EQ(0, g_callback2);
  EXPECT_EQ(std::string(), g_callbackData1);
  EXPECT_EQ(std::string(), g_callbackData2);

  // test preRender
  rendering::Events::cameraPreRender(camera->Name());
  EXPECT_EQ(1, g_callback1);
  EXPECT_EQ(0, g_callback2);
  EXPECT_EQ(camera->Name(), g_callbackData1);
  EXPECT_EQ(std::string(), g_callbackData2);

  // reset
  g_callback1 = 0;
  g_callback2 = 0;
  g_callbackData1 = "";
  g_callbackData2 = "";

  // test postRender
  rendering::Events::cameraPostRender(camera->Name());
  EXPECT_EQ(0, g_callback1);
  EXPECT_EQ(1, g_callback2);
  EXPECT_EQ(std::string(), g_callbackData1);
  EXPECT_EQ(camera->Name(), g_callbackData2);

  // reset
  g_callback1 = 0;
  g_callback2 = 0;
  g_callbackData1 = "";
  g_callbackData2 = "";

  // test preRender and postRender
  rendering::Events::cameraPreRender(camera->Name());
  rendering::Events::cameraPostRender(camera->Name());
  EXPECT_EQ(1, g_callback1);
  EXPECT_EQ(1, g_callback2);
  EXPECT_EQ(camera->Name(), g_callbackData1);
  EXPECT_EQ(camera->Name(), g_callbackData2);

  // reset
  g_callback1 = 0;
  g_callback2 = 0;
  g_callbackData1 = "";
  g_callbackData2 = "";

  // disconnect
  cameraPreRenderConnection.reset();
  cameraPostRenderConnection.reset();

  // fire events and verify no callbacks triggered
  EXPECT_EQ(0, g_callback1);
  EXPECT_EQ(0, g_callback2);
  EXPECT_EQ(std::string(), g_callbackData1);
  EXPECT_EQ(std::string(), g_callbackData2);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
