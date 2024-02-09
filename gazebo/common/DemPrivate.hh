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

#ifndef _GAZEBO_DEM_PRIVATE_HH_
#define _GAZEBO_DEM_PRIVATE_HH_

#include "gazebo/common/SphericalCoordinates.hh"
#include <gazebo/gazebo_config.h>
#include <gazebo/util/system.hh>

#ifdef HAVE_GDAL
# include <gdal_priv.h>
# include <vector>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class DemPrivate DemPrivate.hh common/common.hh
    /// \brief Private data for the Dem class.
    class GZ_COMMON_VISIBLE DemPrivate
    {
      /// \brief A set of associated raster bands.
      public: GDALDataset *dataSet;

      /// \brief A pointer to the band.
      public: GDALRasterBand *band;

      /// \brief Real width of the world in meters.
      public: double worldWidth = 0;

      /// \brief Real height of the world in meters.
      public: double worldHeight = 0;

      /// \brief Terrain's side (after the padding).
      public: unsigned int side;

      /// \brief Minimum elevation in meters.
      public: double minElevation;

      /// \brief Maximum elevation in meters.
      public: double maxElevation;

      /// \brief DEM data converted to be OGRE-compatible.
      public: std::vector<float> demData;

      /// \brief Holds the spherical coordinates object from the world.
      public: common::SphericalCoordinatesPtr sphericalCoordinates =
              boost::make_shared<common::SphericalCoordinates>();
    };
    /// \}
  }
}
#endif
#endif
