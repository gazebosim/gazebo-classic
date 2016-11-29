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

#include <gtest/gtest.h>
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class GpuLaser_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
//                                             //
//  bring up a GpuLaser and exercise API       //
//                                             //
/////////////////////////////////////////////////
TEST_F(GpuLaser_TEST, BasicGpuLaserAPITest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
      scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  gazebo::rendering::GpuLaserPtr laserCam =
    scene->CreateGpuLaser("test_laser", false);

  EXPECT_TRUE(laserCam != NULL);

  // The following tests all the getters and setters
  {
    laserCam->SetNearClip(0.1);
    EXPECT_NEAR(laserCam->NearClip(), 0.1, 1e-6);

    laserCam->SetFarClip(100.0);
    EXPECT_NEAR(laserCam->FarClip(), 100, 1e-6);

    laserCam->SetHorzHalfAngle(1.2);
    EXPECT_NEAR(laserCam->HorzHalfAngle(), 1.2, 1e-6);

    laserCam->SetVertHalfAngle(0.5);
    EXPECT_NEAR(laserCam->VertHalfAngle(), 0.5, 1e-6);

    laserCam->SetIsHorizontal(false);
    EXPECT_FALSE(laserCam->IsHorizontal());

    laserCam->SetHorzFOV(2.4);
    EXPECT_NEAR(laserCam->HorzFOV(), 2.4, 1e-6);

    laserCam->SetVertFOV(1.0);
    EXPECT_NEAR(laserCam->VertFOV(), 1.0, 1e-6);

    laserCam->SetCosHorzFOV(0.2);
    EXPECT_NEAR(laserCam->CosHorzFOV(), 0.2, 1e-6);

    laserCam->SetCosVertFOV(0.1);
    EXPECT_NEAR(laserCam->CosVertFOV(), 0.1, 1e-6);

    laserCam->SetRayCountRatio(0.344);
    EXPECT_NEAR(laserCam->RayCountRatio(), 0.344, 1e-6);

    laserCam->SetCameraCount(4u);
    EXPECT_EQ(laserCam->CameraCount(), 4u);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
