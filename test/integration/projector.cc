/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "images_cmp.h"

using namespace gazebo;
class ProjectorTest : public ServerFixture
{
};

TEST_F(ProjectorTest, Projector)
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
    return;

  Load("worlds/projector.world");
  SpawnCamera("camera_model", "camera_sensor", math::Vector3(-4, 0, 1),
              math::Vector3(0, GZ_DTOR(10), 0));

  unsigned char *img = NULL;
  unsigned int width;
  unsigned int height;
  GetFrame("camera_sensor", &img, width, height);
  ASSERT_EQ(width, static_cast<unsigned int>(320));
  ASSERT_EQ(height, static_cast<unsigned int>(240));

  unsigned int diffMax = 0;
  unsigned int diffSum = 0;
  double diffAvg = 0;
  ImageCompare(img, projector_world_camera,
      width, height, 3, diffMax, diffSum, diffAvg);
  // PrintImage("projector_world_camera", &img, width, height, 3);
  ASSERT_EQ(diffSum, static_cast<unsigned int>(0));
  ASSERT_EQ(diffMax, static_cast<unsigned int>(0));
  ASSERT_EQ(diffAvg, 0.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
