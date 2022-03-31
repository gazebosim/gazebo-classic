/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <tuple>

#include <gtest/gtest.h>

#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test/util.hh"

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
  }
}

/////////////////////////////////////////////////
//                                             //
//  Unit test calculations                     //
//                                             //
/////////////////////////////////////////////////
class GpuLaserInternals_TEST : public gazebo::testing::AutoLogFixture
{
};

TEST_F(GpuLaserInternals_TEST, FindCubeFaceMappingTest)
{
  using namespace rendering;

  GpuLaserCubeMappingPoint p;

  constexpr double numeric_tolerance = 1e-3;

  // ray straight ahead
  ASSERT_NO_THROW(p = rendering::GpuLaser::FindCubeFaceMapping(M_PI_4, 0.));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, p.first);
  EXPECT_NEAR(0.5, p.second.X(), numeric_tolerance);
  EXPECT_NEAR(0.5, p.second.Y(), numeric_tolerance);

  // ray at minimum azimuth
  ASSERT_NO_THROW(p = rendering::GpuLaser::FindCubeFaceMapping(0., 0.));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, p.first);
  EXPECT_NEAR(1.0, p.second.X(), numeric_tolerance);
  EXPECT_NEAR(0.5, p.second.Y(), numeric_tolerance);

  const double corner_elevation = std::atan(M_SQRT1_2);
  constexpr double corner_offset = 1e-4;

  // ray at bottom left rear corner
  ASSERT_NO_THROW(p = rendering::GpuLaser::FindCubeFaceMapping(M_PI + corner_offset, - corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, p.first);
  EXPECT_NEAR(1.0, p.second.X(), numeric_tolerance);
  EXPECT_NEAR(1.0, p.second.Y(), numeric_tolerance);
}

TEST_F(GpuLaserInternals_TEST, ViewingRayTest)
{
  using namespace rendering;

  const double eps = 1e-10;

  ignition::math::Vector3d v;

  // ray with minimum azimuth
  v = rendering::GpuLaser::ViewingRay(0., 0.);
  EXPECT_DOUBLE_EQ(1., v.Length());
  EXPECT_DOUBLE_EQ(1. / M_SQRT2, v.X());
  EXPECT_DOUBLE_EQ(-1. / M_SQRT2, v.Y());
  EXPECT_DOUBLE_EQ(0., v.Z());

  // ray straight ahead
  v = rendering::GpuLaser::ViewingRay(M_PI_4, 0.);
  EXPECT_DOUBLE_EQ(1., v.Length());
  EXPECT_DOUBLE_EQ(1., v.X());
  EXPECT_DOUBLE_EQ(0., v.Y());
  EXPECT_DOUBLE_EQ(0., v.Z());

  // ray straight ahead and elevated
  v = rendering::GpuLaser::ViewingRay(M_PI_4, M_PI_4);
  EXPECT_DOUBLE_EQ(1., v.Length());
  EXPECT_DOUBLE_EQ(1. / M_SQRT2, v.X());
  EXPECT_DOUBLE_EQ(0., v.Y());
  EXPECT_DOUBLE_EQ(1. / M_SQRT2, v.Z());

  // ray left
  v = rendering::GpuLaser::ViewingRay(M_PI_2 + M_PI_4, 0.);
  EXPECT_DOUBLE_EQ(1., v.Length());
  EXPECT_NEAR(0., v.X(), eps);
  EXPECT_DOUBLE_EQ(1., v.Y());
  EXPECT_DOUBLE_EQ(0., v.Z());
}

TEST_F(GpuLaserInternals_TEST, FindCubeFaceTest)
{
  using namespace rendering;

  // at zero elevation
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, GpuLaser::FindCubeFace(0., 0.));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_LEFT_FACE, GpuLaser::FindCubeFace(M_PI_2, 0.));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, GpuLaser::FindCubeFace(M_PI, 0.));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2, 0.));

  // extreme elevation values
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_TOP_FACE, GpuLaser::FindCubeFace(0., M_PI_2));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, GpuLaser::FindCubeFace(0., -M_PI_2));

  const double corner_elevation = std::atan(M_SQRT1_2);
  constexpr double corner_offset = 1e-4;

  // 4 corners of the front face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, GpuLaser::FindCubeFace(corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, GpuLaser::FindCubeFace(M_PI_2 - corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, GpuLaser::FindCubeFace(corner_offset, - corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_FRONT_FACE, GpuLaser::FindCubeFace(M_PI_2 - corner_offset, - corner_elevation + corner_offset));

  // 4 corners of the left face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_LEFT_FACE, GpuLaser::FindCubeFace(M_PI_2 + corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_LEFT_FACE, GpuLaser::FindCubeFace(M_PI - corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_LEFT_FACE, GpuLaser::FindCubeFace(M_PI_2 + corner_offset, - corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_LEFT_FACE, GpuLaser::FindCubeFace(M_PI - corner_offset, - corner_elevation + corner_offset));

  // 4 corners of the rear face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, GpuLaser::FindCubeFace(M_PI + corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2 - corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, GpuLaser::FindCubeFace(M_PI + corner_offset, - corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_REAR_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2 - corner_offset, - corner_elevation + corner_offset));

  // 4 corners of the right face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2 + corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, GpuLaser::FindCubeFace(M_PI + M_PI - corner_offset, corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2 + corner_offset, - corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_RIGHT_FACE, GpuLaser::FindCubeFace(M_PI + M_PI - corner_offset, - corner_elevation + corner_offset));

  // 4 corners of the top face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_TOP_FACE, GpuLaser::FindCubeFace(0.0, corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_TOP_FACE, GpuLaser::FindCubeFace(M_PI_2, corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_TOP_FACE, GpuLaser::FindCubeFace(M_PI, corner_elevation + corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_TOP_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2, corner_elevation + corner_offset));

  // 4 corners of the bottom face
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, GpuLaser::FindCubeFace(0.0, - corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, GpuLaser::FindCubeFace(M_PI_2, - corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, GpuLaser::FindCubeFace(M_PI, - corner_elevation - corner_offset));
  EXPECT_EQ(GpuLaserCubeFaceId::CUBE_BOTTOM_FACE, GpuLaser::FindCubeFace(M_PI + M_PI_2, - corner_elevation - corner_offset));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
