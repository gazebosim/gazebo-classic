/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_HEIGHTMAPSHAPEPRIVATE_HH_
#define _GAZEBO_PHYSICS_HEIGHTMAPSHAPEPRIVATE_HH_
#include <vector>
#include <string>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/ImageHeightmap.hh"
#include "gazebo/common/HeightmapData.hh"
#include "gazebo/common/Dem.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/ShapePrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for HeightmapShape.
    class HeightmapShapePrivate : public ShapePrivate
    {
      /// \brief Lookup table of heights.
      public: std::vector<float> heights;

      /// \brief Image used to generate the heights.
      public: common::ImageHeightmap img;

      /// \brief HeightmapData used to generate the heights.
      public: common::HeightmapData *heightmapData;

      /// \brief Size of the height lookup table.
      public: unsigned int vertSize;

      /// \brief True to flip the heights along the y direction.
      public: bool flipY;

      /// \brief The amount of subsampling. Default is 2.
      public: int subSampling;

      /// \brief Transportation node.
      public: transport::NodePtr node;

      /// \brief Subscriber to request messages.
      public: transport::SubscriberPtr requestSub;

      /// \brief Publisher for request response messages.
      public: transport::PublisherPtr responsePub;

      /// \brief File format of the heightmap
      public: std::string fileFormat;

      /// \brief Terrain size
      public: ignition::math::Vector3d heightmapSize;

      #ifdef HAVE_GDAL
      /// \brief DEM used to generate the heights.
      public: common::Dem dem;
      #endif
    };
  }
}
#endif
