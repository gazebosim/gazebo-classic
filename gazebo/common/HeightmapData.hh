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

#ifndef _GAZEBO_HEIGHTMAPDATA_HH_
#define _GAZEBO_HEIGHTMAPDATA_HH_

#include <vector>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class HeightmapData HeightmapData.hh common/common.hh
    /// \brief Encapsulates a generic heightmap data file.
    class GZ_COMMON_VISIBLE HeightmapData
    {
      /// \brief Destructor.
      public: virtual ~HeightmapData() {}

      /// \brief Create a lookup table of the terrain's height.
      /// \param[in] _subsampling Multiplier used to increase the resolution.
      /// Ex: A subsampling of 2 in a terrain of 129x129 means that the height
      /// vector will be 257 * 257.
      /// \param[in] _vertSize Number of points per row.
      /// \param[in] _size Real dimmensions of the terrain.
      /// \param[in] _scale Vector3 used to scale the height.
      /// \param[in] _flipY If true, it inverts the order in which the vector
      /// is filled.
      /// \param[out] _heights Vector containing the terrain heights.
      /// \deprecated See FillHeightMap() that accepts
      /// ignition::math::Vector3d objects.
      public: virtual void FillHeightMap(int _subSampling,
          unsigned int _vertSize, const math::Vector3 &_size,
          const math::Vector3 &_scale, bool _flipY,
          std::vector<float> &_heights) GAZEBO_DEPRECATED(6.0) = 0;

      /// \brief Create a lookup table of the terrain's height.
      /// \param[in] _subsampling Multiplier used to increase the resolution.
      /// Ex: A subsampling of 2 in a terrain of 129x129 means that the height
      /// vector will be 257 * 257.
      /// \param[in] _vertSize Number of points per row.
      /// \param[in] _size Real dimmensions of the terrain.
      /// \param[in] _scale Vector3 used to scale the height.
      /// \param[in] _flipY If true, it inverts the order in which the vector
      /// is filled.
      /// \param[out] _heights Vector containing the terrain heights.
      public: virtual void FillHeightMap(int _subSampling,
          unsigned int _vertSize, const ignition::math::Vector3d &_size,
          const ignition::math::Vector3d &_scale, bool _flipY,
          std::vector<float> &_heights) = 0;

      /// \brief Get the terrain's height.
      /// \return The terrain's height.
      public: virtual unsigned int GetHeight() const = 0;

      /// \brief Get the terrain's width.
      /// \return The terrain's width.
      public: virtual unsigned int GetWidth() const = 0;

      /// \brief Get the maximum terrain's elevation.
      /// \return The maximum terrain's elevation.
      public: virtual float GetMaxElevation() const = 0;
    };
    /// \}
  }
}
#endif
