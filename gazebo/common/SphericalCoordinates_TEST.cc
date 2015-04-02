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

#include "gazebo/common/Console.hh"
#include "gazebo/common/SphericalCoordinates.hh"
#include "test/util.hh"

using namespace gazebo;

class SphericalCoordinatesTest : public gazebo::testing::AutoLogFixture { };

//////////////////////////////////////////////////
// Test different constructors, default parameters
TEST_F(SphericalCoordinatesTest, Constructor)
{
  // Default surface type
  common::SphericalCoordinates::SurfaceType st =
    common::SphericalCoordinates::EARTH_WGS84;

  // No arguments, default parameters
  {
    common::SphericalCoordinates sc;
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.GetLatitudeReference(), math::Angle());
    EXPECT_EQ(sc.GetLongitudeReference(), math::Angle());
    EXPECT_EQ(sc.GetHeadingOffset(), math::Angle());
    EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);
  }

  // SurfaceType argument, default parameters
  {
    common::SphericalCoordinates sc(st);
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.GetLatitudeReference(), math::Angle());
    EXPECT_EQ(sc.GetLongitudeReference(), math::Angle());
    EXPECT_EQ(sc.GetHeadingOffset(), math::Angle());
    EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);
  }

  // All arguments
  {
    math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    common::SphericalCoordinates sc(st, lat, lon, elev, heading);
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.GetLatitudeReference(), lat);
    EXPECT_EQ(sc.GetLongitudeReference(), lon);
    EXPECT_EQ(sc.GetHeadingOffset(), heading);
    EXPECT_NEAR(sc.GetElevationReference(), elev, 1e-6);
  }
}

//////////////////////////////////////////////////
// SurfaceType Convert function
TEST_F(SphericalCoordinatesTest, Convert)
{
  // Default surface type
  common::SphericalCoordinates::SurfaceType st =
    common::SphericalCoordinates::EARTH_WGS84;

  EXPECT_EQ(common::SphericalCoordinates::Convert("EARTH_WGS84"), st);
}

//////////////////////////////////////////////////
// Test Set functions
TEST_F(SphericalCoordinatesTest, SetFunctions)
{
  // Default surface type
  common::SphericalCoordinates::SurfaceType st =
    common::SphericalCoordinates::EARTH_WGS84;

  // Default parameters
  common::SphericalCoordinates sc;
  EXPECT_EQ(sc.GetSurfaceType(), st);
  EXPECT_EQ(sc.GetLatitudeReference(), math::Angle());
  EXPECT_EQ(sc.GetLongitudeReference(), math::Angle());
  EXPECT_EQ(sc.GetHeadingOffset(), math::Angle());
  EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);

  {
    math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    sc.SetSurfaceType(st);
    sc.SetLatitudeReference(lat);
    sc.SetLongitudeReference(lon);
    sc.SetHeadingOffset(heading);
    sc.SetElevationReference(elev);

    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.GetLatitudeReference(), lat);
    EXPECT_EQ(sc.GetLongitudeReference(), lon);
    EXPECT_EQ(sc.GetHeadingOffset(), heading);
    EXPECT_NEAR(sc.GetElevationReference(), elev, 1e-6);
  }
}

//////////////////////////////////////////////////
// Test coordinate transformations
TEST_F(SphericalCoordinatesTest, CoordinateTransforms)
{
  // Default surface type
  common::SphericalCoordinates::SurfaceType st =
    common::SphericalCoordinates::EARTH_WGS84;

  {
    // Parameters
    math::Angle lat(0.3), lon(-1.2), heading(math::Angle::HalfPi);
    double elev = 354.1;
    common::SphericalCoordinates sc(st, lat, lon, elev, heading);

    // Check GlobalFromLocal with heading offset of 90 degrees
    {
      // local frame
      math::Vector3 xyz;
      // east, north, up
      math::Vector3 enu;

      xyz.Set(1, 0, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.y, xyz.x, 1e-6);
      EXPECT_NEAR(enu.x, -xyz.y, 1e-6);

      xyz.Set(0, 1, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.y, xyz.x, 1e-6);
      EXPECT_NEAR(enu.x, -xyz.y, 1e-6);

      xyz.Set(1, -1, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.y, xyz.x, 1e-6);
      EXPECT_NEAR(enu.x, -xyz.y, 1e-6);

      xyz.Set(2243.52334, 556.35, 435.6553);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.y, xyz.x, 1e-6);
      EXPECT_NEAR(enu.x, -xyz.y, 1e-6);
    }

    // Check SphericalFromLocal
    {
      // local frame
      math::Vector3 xyz;
      // spherical coordinates
      math::Vector3 sph;

      // No offset
      xyz.Set(0, 0, 0);
      sph = sc.SphericalFromLocal(xyz);
      // latitude
      EXPECT_NEAR(sph.x, lat.Degree(), 1e-6);
      // longitude
      EXPECT_NEAR(sph.y, lon.Degree(), 1e-6);
      // elevation
      EXPECT_NEAR(sph.z, elev, 1e-6);

      // 200 km offset in x (pi/2 heading offset means North)
      xyz.Set(2e5, 0, 0);
      sph = sc.SphericalFromLocal(xyz);
      // increase in latitude about 1.8 degrees
      EXPECT_NEAR(sph.x, lat.Degree() + 1.8, 0.008);
      // no change in longitude
      EXPECT_NEAR(sph.y, lon.Degree(), 1e-6);
      // no change in elevation
      EXPECT_NEAR(sph.z, elev, 1e-6);
    }
  }
}

//////////////////////////////////////////////////
// Test distance
TEST_F(SphericalCoordinatesTest, Distance)
{
  math::Angle latA, longA, latB, longB;
  latA.SetFromDegree(46.250944);
  longA.SetFromDegree(-122.249972);
  latB.SetFromDegree(46.124953);
  longB.SetFromDegree(-122.251683);
  double d = common::SphericalCoordinates::Distance(latA, longA, latB, longB);

  EXPECT_NEAR(14002, d, 20);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
