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
  // Constants
  const double g_EarthWGS84AxisEquatorial = 6378137.0;
  const double g_EarthWGS84AxisPolar = 6356752.314245;
  const double g_EarthWGS84Flattening = 1.0/298.257223563;
  const double g_EarthRadius = 6371000.0;

  const double g_MoonRadius = 1737400.0;
  const double g_MoonAxisEquatorial = 1738100.0;
  const double g_MoonAxisPolar = 1736000.0;
  const double g_MoonFlattening = 0.0012;

  // Default surface type
  common::SphericalCoordinates::SurfaceType st =
    common::SphericalCoordinates::EARTH_WGS84;

  // No arguments, default parameters
  {
    common::SphericalCoordinates sc;
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
    EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);

    EXPECT_NEAR(sc.SurfaceRadius(), g_EarthRadius, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisEquatorial(),
        g_EarthWGS84AxisEquatorial, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisPolar(),
        g_EarthWGS84AxisPolar, 1e-6);
    EXPECT_NEAR(sc.SurfaceFlattening(),
        g_EarthWGS84Flattening, 1e-6);
  }

  // SurfaceType argument, default parameters
  {
    common::SphericalCoordinates sc(st);
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
    EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);

    EXPECT_NEAR(sc.SurfaceRadius(), g_EarthRadius, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisEquatorial(),
        g_EarthWGS84AxisEquatorial, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisPolar(),
        g_EarthWGS84AxisPolar, 1e-6);
    EXPECT_NEAR(sc.SurfaceFlattening(),
        g_EarthWGS84Flattening, 1e-6);
  }

  // All arguments
  {
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    common::SphericalCoordinates sc(st, lat, lon, elev, heading);
    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.LatitudeReference(), lat);
    EXPECT_EQ(sc.LongitudeReference(), lon);
    EXPECT_EQ(sc.HeadingOffset(), heading);
    EXPECT_NEAR(sc.GetElevationReference(), elev, 1e-6);

    EXPECT_NEAR(sc.SurfaceRadius(), g_EarthRadius, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisEquatorial(),
        g_EarthWGS84AxisEquatorial, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisPolar(),
        g_EarthWGS84AxisPolar, 1e-6);
    EXPECT_NEAR(sc.SurfaceFlattening(),
        g_EarthWGS84Flattening, 1e-6);
  }

  // For Moon surface
  {
    common::SphericalCoordinates sc(
        common::SphericalCoordinates::MOON_SCS);
    EXPECT_EQ(sc.GetSurfaceType(),
        common::SphericalCoordinates::MOON_SCS);
    EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
    EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
    EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);

    EXPECT_NEAR(sc.SurfaceRadius(), g_MoonRadius, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisEquatorial(),
        g_MoonAxisEquatorial, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisPolar(),
        g_MoonAxisPolar, 1e-6);
    EXPECT_NEAR(sc.SurfaceFlattening(),
        g_MoonFlattening, 1e-6);
  }

  // Moon surface with arguments
  {
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    common::SphericalCoordinates sc(
        common::SphericalCoordinates::MOON_SCS,
        lat, lon, elev, heading);
    EXPECT_EQ(sc.GetSurfaceType(),
        common::SphericalCoordinates::MOON_SCS);
    EXPECT_EQ(sc.LatitudeReference(), lat);
    EXPECT_EQ(sc.LongitudeReference(), lon);
    EXPECT_EQ(sc.HeadingOffset(), heading);
    EXPECT_NEAR(sc.GetElevationReference(), elev, 1e-6);

    EXPECT_NEAR(sc.SurfaceRadius(), g_MoonRadius, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisEquatorial(),
        g_MoonAxisEquatorial, 1e-6);
    EXPECT_NEAR(sc.SurfaceAxisPolar(),
        g_MoonAxisPolar, 1e-6);
    EXPECT_NEAR(sc.SurfaceFlattening(),
        g_MoonFlattening, 1e-6);
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

  // For Moon surface type
  st = common::SphericalCoordinates::MOON_SCS;
  EXPECT_EQ(common::SphericalCoordinates::Convert("MOON_SCS"), st);
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
  EXPECT_EQ(sc.LatitudeReference(), ignition::math::Angle());
  EXPECT_EQ(sc.LongitudeReference(), ignition::math::Angle());
  EXPECT_EQ(sc.HeadingOffset(), ignition::math::Angle());
  EXPECT_NEAR(sc.GetElevationReference(), 0.0, 1e-6);

  {
    ignition::math::Angle lat(0.3), lon(-1.2), heading(0.5);
    double elev = 354.1;
    sc.SetSurfaceType(st);
    sc.SetLatitudeReference(lat);
    sc.SetLongitudeReference(lon);
    sc.SetHeadingOffset(heading);
    sc.SetElevationReference(elev);

    EXPECT_EQ(sc.GetSurfaceType(), st);
    EXPECT_EQ(sc.LatitudeReference(), lat);
    EXPECT_EQ(sc.LongitudeReference(), lon);
    EXPECT_EQ(sc.HeadingOffset(), heading);
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
    ignition::math::Angle lat(0.3), lon(-1.2),
      heading(ignition::math::Angle::HalfPi);
    double elev = 354.1;
    common::SphericalCoordinates sc(st, lat, lon, elev, heading);

    // Check GlobalFromLocal with heading offset of 90 degrees
    {
      // local frame
      ignition::math::Vector3d xyz;
      // east, north, up
      ignition::math::Vector3d enu;

      xyz.Set(1, 0, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobal(enu));

      xyz.Set(0, 1, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobal(enu));

      xyz.Set(1, -1, 0);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobal(enu));

      xyz.Set(2243.52334, 556.35, 435.6553);
      enu = sc.GlobalFromLocal(xyz);
      EXPECT_NEAR(enu.Y(), xyz.X(), 1e-6);
      EXPECT_NEAR(enu.X(), -xyz.Y(), 1e-6);
      EXPECT_EQ(xyz, sc.LocalFromGlobal(enu));
    }

    // Check SphericalFromLocal
    {
      // local frame
      ignition::math::Vector3d xyz;
      // spherical coordinates
      ignition::math::Vector3d sph;

      // No offset
      xyz.Set(0, 0, 0);
      sph = sc.SphericalFromLocal(xyz);
      // latitude
      EXPECT_NEAR(sph.X(), lat.Degree(), 1e-6);
      // longitude
      EXPECT_NEAR(sph.Y(), lon.Degree(), 1e-6);
      // elevation
      EXPECT_NEAR(sph.Z(), elev, 1e-6);

      // 200 km offset in x (pi/2 heading offset means North). We use
      // SphericalFromLocal, which means that xyz is a linear movement on
      // a plane (not along the curvature of Earth). This will result in
      // a large height offset.
      xyz.Set(2e5, 0, 0);
      sph = sc.SphericalFromLocal(xyz);
      // increase in latitude about 1.8 degrees
      EXPECT_NEAR(sph.X(), lat.Degree() + 1.8, 0.008);
      // no change in longitude
      EXPECT_NEAR(sph.Z(), 3507.024791, 1e-6);

      ignition::math::Vector3d xyz2 = sc.LocalFromSpherical(sph);
      EXPECT_EQ(xyz, xyz2);
    }

    // Check position projection
    {
      // WGS84 coordinate obtained from online mapping software
      // > gdaltransform -s_srs WGS84 -t_srs EPSG:4978
      // > latitude longitude altitude
      // > X Y Z
      ignition::math::Vector3d tmp;
      ignition::math::Vector3d osrf_s(37.3877349, -122.0651166, 32.0);
      ignition::math::Vector3d osrf_e(
          -2693701.91434394, -4299942.14687992, 3851691.0393571);
      ignition::math::Vector3d goog_s(37.4216719, -122.0821853, 30.0);
      ignition::math::Vector3d goog_e(
          -2693766.71906146, -4297199.59926038, 3854681.81878812);

      // Local tangent plane coordinates (ENU = GLOBAL) coordinates of
      // Google when OSRF is taken as the origin:
      // > proj +ellps=WGS84  +proj=tmerc
      // +lat_0=37.3877349 +lon_0=-122.0651166 +k=1 +x_0=0 +y_0=0
      // > -122.0821853 37.4216719 (LON,LAT)
      // > -1510.88 3766.64 (EAST,NORTH)
      ignition::math::Vector3d vec(-1510.88, 3766.64, -3.29);

      // Convert degrees to radians
      osrf_s.X() *= 0.0174532925;
      osrf_s.Y() *= 0.0174532925;

      // Set the ORIGIN to be the Open Source Robotics Foundation
      common::SphericalCoordinates sc2(st, ignition::math::Angle(osrf_s.X()),
          ignition::math::Angle(osrf_s.Y()), osrf_s.Z(), 0.0);

      // Check that SPHERICAL -> ECEF works
      tmp = sc2.PositionTransform(osrf_s,
          common::SphericalCoordinates::SPHERICAL,
          common::SphericalCoordinates::ECEF);

      EXPECT_NEAR(tmp.X(), osrf_e.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), osrf_e.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), osrf_e.Z(), 1e-2);

      // Check that ECEF -> SPHERICAL works
      tmp = sc2.PositionTransform(tmp,
          common::SphericalCoordinates::ECEF,
          common::SphericalCoordinates::SPHERICAL);

      EXPECT_NEAR(tmp.X(), osrf_s.X(), 1e-2);
      EXPECT_NEAR(tmp.Y(), osrf_s.Y(), 1e-2);
      EXPECT_NEAR(tmp.Z(), osrf_s.Z(), 1e-2);

      // Check that SPHERICAL -> LOCAL works
      tmp = sc2.LocalFromSpherical(goog_s);
      EXPECT_NEAR(tmp.X(), vec.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), vec.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), vec.Z(), 1e-2);

      // Check that SPHERICAL -> LOCAL -> SPHERICAL works
      tmp = sc2.SphericalFromLocal(tmp);
      EXPECT_NEAR(tmp.X(), goog_s.X(), 8e-2);
      EXPECT_NEAR(tmp.Y(), goog_s.Y(), 8e-2);
      EXPECT_NEAR(tmp.Z(), goog_s.Z(), 1e-2);
    }
  }
}

//////////////////////////////////////////////////
// Test distance
TEST_F(SphericalCoordinatesTest, Distance)
{
  ignition::math::Angle latA, longA, latB, longB;
  latA.Degree(46.250944);
  longA.Degree(-122.249972);
  latB.Degree(46.124953);
  longB.Degree(-122.251683);

  // Calculating distance using the static method.
  double d1 = common::SphericalCoordinates::Distance(latA, longA, latB, longB);
  EXPECT_NEAR(14002, d1, 20);

  // Using the non static method. The default surface type is EARTH_WGS84.
  common::SphericalCoordinates defaultEarthSC = common::SphericalCoordinates();
  double d2 = defaultEarthSC.DistanceBetweenPoints(latA, longA, latB, longB);
  EXPECT_NEAR(d1, d2, 0.1);

  common::SphericalCoordinates earthSC = common::SphericalCoordinates(
      common::SphericalCoordinates::EARTH_WGS84);
  double d3 = earthSC.DistanceBetweenPoints(latA, longA, latB, longB);
  EXPECT_NEAR(d2, d3, 0.1);

  // Setting the surface type as Moon.
  common::SphericalCoordinates moonSC = common::SphericalCoordinates(
      common::SphericalCoordinates::MOON_SCS);
  double d4 = moonSC.DistanceBetweenPoints(latA, longA, latB, longB);
  EXPECT_NEAR(3820, d4, 5);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
