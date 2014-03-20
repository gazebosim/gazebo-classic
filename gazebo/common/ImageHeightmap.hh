/*
 * Copyright 2012-2014 Open Source Robotics Foundation
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

#ifndef _IMAGE_HEIGHTMAP_DATA_HH_
#define _IMAGE_HEIGHTMAP_DATA_HH_

#include <string>
#include <vector>
#include "gazebo/common/HeightmapData.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ImageHeightmap ImageHeightmap.hh common/common.hh
    /// \brief Encapsulates an image that will be interpreted as a heightmap.
    class GAZEBO_VISIBLE ImageHeightmap
      : public gazebo::common::HeightmapData
    {
      /// \brief Constructor
      /// \param[in] _filename the path to the image
      public: ImageHeightmap();

      /// \brief Load an image file as a heightmap.
      /// \param[in] _filename the path to the image file.
      /// \return True when the operation succeeds to open a file.
      public: int Load(const std::string &_filename="");

      // Documentation inherited.
      public: void FillHeightMap(int _subSampling, unsigned int _vertSize,
          const math::Vector3 &_size, const math::Vector3 &_scale, bool _flipY,
          std::vector<float> &_heights);

      /// \brief Get the full filename of the image
      /// \return The filename used to load the image
      public: std::string GetFilename() const;

      // Documentation inherited.
      public: unsigned int GetHeight() const;

      // Documentation inherited.
      public: unsigned int GetWidth() const;

      // Documentation inherited.
      public: float GetMaxElevation() const;

      /// \brief Image containing the heightmap data.
      private: gazebo::common::Image img;
    };
    /// \}
  }
}
#endif
