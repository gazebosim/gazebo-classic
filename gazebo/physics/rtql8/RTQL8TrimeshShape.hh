/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _RTQL8TRIMESHSHAPE_HH_
#define _RTQL8TRIMESHSHAPE_HH_

#include "gazebo/physics/TrimeshShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Triangle mesh collision.
    class RTQL8TrimeshShape : public TrimeshShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision object.
      public: explicit RTQL8TrimeshShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~RTQL8TrimeshShape();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Update();

      /// \brief Transform matrix.
      //private: dReal transform[16*2];

      /// \brief Transform matrix index.
      private: int transformIndex;

      /// \brief Array of vertex values.
      private: float *vertices;

      /// \brief Array of index values.
      private: int *indices;

      /// \brief RTQL8 trimesh data.
      //private: dTriMeshDataID odeData;
    };
  }
}
#endif
