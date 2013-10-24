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

#ifndef _HEIGHTMAPDATA_HH_
#define _HEIGHTMAPDATA_HH_

#include <vector>
#include "gazebo/common/Color.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class HeighmapData HeighmapData.hh common/common.hh
    /// \brief Encapsulates a generic heightmap data file
    class HeightmapData
    {
      /// \brief Destructor
      public: virtual ~HeightmapData() {}

      /// \brief Create a lookup table of the terrain's height
      /// \param[in] _subsampling
      /// \param[in] _vertSize
      /// \param[in] _size
      /// \param[in] _scale
      /// \param[in] _flipY
      /// \param[out] _heights
      public: virtual void FillHeightMap(int _subSampling,
          unsigned int _vertSize, const math::Vector3 &_size,
          const math::Vector3 &_scale, bool _flipY,
          std::vector<float> &_heights) = 0;

      /// \brief Get the size of one pixel in bits
      /// \return The BPP of the image
      public: virtual unsigned int GetBPP() const = 0;

      /// \brief Get the height
      /// \return The image height
      public: virtual unsigned int GetHeight() const = 0;

      /// \brief Get the max color
      /// \return The max color
      public: virtual Color GetMaxColor() = 0;

      // \brief Get the size of a row of pixel
      /// \return The pitch of the image
      public: virtual int GetPitch() const = 0;

      /// \brief Get the width
      /// \return The image width
      public: virtual unsigned int GetWidth() const = 0;
    };
    /// \}
  }
}

#endif
