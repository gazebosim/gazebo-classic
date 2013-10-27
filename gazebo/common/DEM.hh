/*
 * Copyright 2012-2013 Open Source Robotics Foundation
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

#ifndef _DEM_HH_
#define _DEM_HH_

#include <gazebo/gazebo_config.h>

#ifdef HAVE_GDAL
# include <gdal/gdal_priv.h>
# include <string>
# include <vector>

# include "gazebo/common/HeightmapData.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class DEM DEM.hh common/common.hh
    /// \brief Encapsulates a DEM (Digital Elevation Model) file.
    /// Examples of DEM formats are GeoTIFF or HFA/Erdas.
    class DEM : public HeightmapData
    {
      /// \brief Constructor.
      public: DEM();

      /// \brief Destructor.
      public: virtual ~DEM();

      /// \brief Load a DEM file.
      /// \param[in] _filename the path to the terrain file.
      public: void Load(const std::string &_filename="");

      /// \brief Get the terrain file as a data array.
      /// \param[out] _data Pointer to a NULL array of char.
      /// \param[out] _count The resulting data array size.
      private: void GetData(float **_data, unsigned int &_count) const;

      /// \brief Get the terrain height. Due to the Ogre constrains, this
      /// value will be equal to GetWidth() and a power of two plus one. The
      /// value returned might be different that the original DEM height because
      /// GetData() adds the padding if necessary.
      /// \return The terrain height satisfying the ogre constrains (squared
      /// terrain with a height value that must be a power of two plus one).
      public: unsigned int GetHeight() const;

      /// \brief Get the terrain width. Due to the Ogre constrains, this
      /// value will be equal to GetHeight() and a power of two plus one. The
      /// value returned might be different that the original DEM width because
      /// GetData() adds the padding if necessary.
      /// \return The terrain width satisfying the ogre constrains (squared
      /// terrain with a width value that must be a power of two plus one).
      public: unsigned int GetWidth() const;

      /// \brief Get the real world width in meters.
      public: double GetWorldWidth() const;

      /// \brief Get the real world height in meters.
      public: double GetWorldHeight() const;

      /// \brief Get the minimum terrain's elevation.
      /// \return The minimum elevation.
      public: float GetMinElevation() const;

      /// \brief Get the maximum terrain's elevation.
      /// \return The maximum elevation.
      public: float GetMaxElevation() const;

      /// \brief Create a lookup table of the terrain's height
      /// \param[in] _subsampling Multiplier used to increase the resolution.
      /// Ex: A subsampling of 2 in a terrain of 129x129 means that the height
      /// vector will be 257 * 257.
      /// \param[in] _vertSize Number of points per row.
      /// \param[in] _size Real dimmensions of the terrain.
      /// \param[in] _scale Vector3 used to scale the height.
      /// \param[in] _flipY If true, it inverts the order in which the vector
      /// is filled.
      /// \param[out] _heights Vector containing the terrain heights.
      public: void FillHeightMap(int _subSampling, unsigned int _vertSize,
          const math::Vector3 &_size, const math::Vector3 &_scale, bool _flipY,
          std::vector<float> &_heights);

      /// \brief Get the georeferenced coordinates (lat, long) of a terrain's
      /// pixel.
      /// \param[in] _x X coordinate of the terrain.
      /// \param[in] _y Y coordinate of the terrain.
      /// \param[out] _xGeo Georeferenced longitude.
      /// \param[out] _yGeo Georeferenced latitude.
      private: void GetGeoReference(double _x, double _y,
                                    double &_xGeo, double &_yGeo);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude.
      /// Example: _latAdeg = 38.0016667 and _lonAdeg = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// \param[in] _latAdeg Latitude of point A.
      /// \param[in] _longAdef Longitude of point A.
      /// \param[in] _latBdeg Latitude of point B.
      /// \param[in] _longBdeg Longitude of point B.
      /// \return Distance in meters.
      private: double Distance(double _latAdeg, double _lonAdeg,
                               double _LatBdeg, double _lonBdeg);

      /// \brief A set of associated raster bands.
      private: GDALDataset *dataSet;

      /// \brief A pointer to the band.
      private: GDALRasterBand *band;

      /// \brief Real width of the world in meters.
      private: double worldWidth;

      /// \brief Real height of the world in meters.
      private: double worldHeight;

      /// \brief Terrain's width (after the padding).
      private: int width;

      /// \brief Terrain's height (after the padding).
      private: int height;

      /// \brief Minimum height in meters.
      private: double minElevation;

      /// \brief Maximum height in meters.
      private: double maxElevation;
    };
    /// \}
  }
}
#endif
#endif
