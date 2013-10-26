/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _SDTS_HH_
#define _SDTS_HH_

#include <gazebo/gazebo_config.h>

#ifdef HAVE_GDAL
# include <gdal/gdal_priv.h>
# include <string>
# include <vector>


# include "gazebo/common/Color.hh"
# include "gazebo/common/Exception.hh"
# include "gazebo/common/HeightmapData.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class SDTS SDTS.hh common/common.hh
    /// \brief Encapsulates an SDTS (Spatial Data Transfer standard GIS file.
    /// Examples of SDTS formats are GeoTIFF or HFA/Erdas.
    class SDTS : public HeightmapData
    {
      /// \brief Constructor.
      public: SDTS();

      /// \brief Destructor.
      public: virtual ~SDTS();

      /// \brief Load a DEM file.
      /// \param[in] _filename the path to the terrain file.
      public: void Load(const std::string &_filename="");

      /// \brief Get the size of each point in bytes after using GetData().
      /// \return The BPP of each point returned after GetData().
      public: unsigned int GetBPP() const;

      /// \brief Get the terrain file as a data array.
      /// \param[out] _data Pointer to a NULL array of char.
      /// \param[out] _count The resulting data array size.
      private: void GetData(float **_data,
                            unsigned int &_count) const;

      /// \brief Get the height.
      /// \return The terrain height.
      public: unsigned int GetHeight() const;

      public: float GetMin();

      public: float GetMax();

      /// \brief Get the max color.
      /// \return The max color.
      public: float GetMaxValue();

      // \brief Get the size of a row of data after using GetData().
      /// \return The pitch of the array returned by GetData().
      public: int GetPitch() const;

      /// \brief Get the width.
      /// \return The terrain width.
      public: unsigned int GetWidth() const;

      /// \brief Get the real world width in meters.
      public: double GetWorldWidth();

      /// \brief Get the real world height in meters.
      public: double GetWorldHeight();

      /// \brief Create a lookup table of the terrain's height
      /// \param[in] _subsampling
      /// \param[in] _vertSize
      /// \param[in] _size
      /// \param[in] _scale
      /// \param[in] _flipY
      /// \param[out] _heights
      public: void FillHeightMap(int _subSampling, unsigned int _vertSize,
          const math::Vector3 &_size, const math::Vector3 &_scale, bool _flipY,
          std::vector<float> &_heights);

      /// \brief Get the georeferenced coordinates (lat, long) of a terrain's
      /// pixel.
      /// \param[in] _pixel X coordinate of the terrain.
      /// \param[in] _line Y coordinate of the terrain.
      /// \param[out] _xGeo Georeferenced longitude.
      /// \param[out] _yGeo Georeferenced latitude.
      private: void GetGeoReference(double _pixel, double _line,
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
      private: double OgrDistance(double _latAdeg, double _lonAdeg,
                                  double _LatBdeg, double _lonBdeg);

      /// \brief Get arccos of a value making some extra checks.
      /// \param[in] _x Input value
      /// \return arccos of input value
      private: static double SafeAcos(double _x);

      /// \brief A set of associated raster bands.
      private: GDALDataset *poDataset;

      /// \brief Real width of the world in meters.
      private: double worldWidth;

      /// \brief Real height of the world in meters.
      private: double worldHeight;

      /// \brief Data type contained in every cell of the raster data.
      private: GDALDataType dataType;

      /// \brief Size of this->dataType in bytes.
      private: int numBytesPerPoint;
    };
    /// \}
  }
}
#endif
#endif
