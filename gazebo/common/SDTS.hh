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

#include <string>
#include <gdal/gdal_priv.h>

#include "gazebo/common/Color.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/HeightmapData.hh"

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
      /// \param[in] _filename the path to the terrain file.
      public: SDTS(const std::string &_filename="");

      /// \brief Destructor.
      public: virtual ~SDTS();

      /// \brief Extend the current terrain by joining a second terrain. This
      /// operation modifies the current terrain width and height. The new width
      /// and height are calculated by choosing the minimum bounding box that
      /// covers the two terrains and satisfy Ogre restrictions (size is a power
      /// of two plus one).
      /// \param[in] _terrain Terrain to add.
      // public: void Add(const SDTS &_terrain);

      /// \brief Get the size of each point in bytes after using GetData().
      /// \return The BPP of each point returned after GetData().
      public: unsigned int GetBPP() const;

      /// \brief Get the terrain file as a data array.
      /// \param[out] _data Pointer to a NULL array of char.
      /// \param[out] _count The resulting data array size.
      public: void GetData(unsigned char **_data,
                           unsigned int &_count) const;

      /// \brief Get the height.
      /// \return The terrain height.
      public: unsigned int GetHeight() const;

      /// \brief Get the max color.
      /// \return The max color.
      public: Color GetMaxColor();

      // \brief Get the size of a row of data after using GetData().
      /// \return The pitch of the array returned by GetData().
      public: int GetPitch() const;

      /// \brief Get the width.
      /// \return The terrain width.
      public: unsigned int GetWidth() const;


      public: double GetWorldWidth();

      public: double GetWorldHeight();

      private: void GetGeoReference(double _pixel, double _line,
                                    double &_xGeo, double &_yGeo);

      private: double OgrDistance(double _latAdeg, double _lonAdeg,
                                  double _LatBdeg, double _lonBdeg);

      private: static double SafeAcos(double _x);

      /// \brief A set of associated raster bands.
      private: GDALDataset *poDataset;

      private: double worldWidth;

      private: double worldHeight;
    };
    /// \}
  }
}
#endif
