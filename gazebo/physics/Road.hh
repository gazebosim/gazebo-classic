/*
 * Copyright 2011 Nate Koenig
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

#ifndef _ROAD_HH_
#define _ROAD_HH_

#include <vector>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/Base.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief for building a Road from SDF
    class Road : public Base
    {
      /// \brief Constructor
      public: Road(BasePtr _parent);

      /// \brief Destructor
      public: virtual ~Road();

      /// \brief Load the road from SDF
      public: void Load(sdf::ElementPtr _elem);

      public: virtual void Init();

      private: double width;
      private: std::vector<math::Vector3> points;
      private: transport::NodePtr node;
      private: transport::PublisherPtr roadPub;
    };
    /// \}
  }
}
#endif
