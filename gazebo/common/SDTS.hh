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
#include <gdal/cpl_conv.h> // for CPLMalloc()

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
    /// \brief Encapsulates an SDTS (Spatial Data Transfer standard GIS file
    class SDTS : public HeightmapData
    {
      /// \brief Constructor
      /// \param[in] _filename the path to the image
      public: SDTS(const std::string &_filename="");

      /// \brief Destructor
      public: virtual ~SDTS();

      /// \brief Get the size of one point in bits
      /// \return The BPP of the image
      public: unsigned int GetBPP() const;

      /// \brief Get the heightmap as a data array
      /// \param[out] _data Pointer to a NULL array of char.
      /// \param[out] _count The resulting data array size
      public: void GetData(unsigned char **_data,
                           unsigned int &_count) const;

      /// \brief Get the height
      /// \return The pixel height
      public: unsigned int GetHeight() const;

      /// \brief Get the max color
      /// \return The max color
      public: Color GetMaxColor();

      // \brief Get the size of a row of data
      /// \return The pitch of the file
      public: int GetPitch() const;

      /// \brief Get the width
      /// \return The pixel width
      public: unsigned int GetWidth() const;

      private: GDALDataset *poDataset;
    };
    /// \}
  }
}
#endif
