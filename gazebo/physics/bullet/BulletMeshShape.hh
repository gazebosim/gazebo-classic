/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _GAZEBO_BULLETMESHSHAPE_HH_
#define _GAZEBO_BULLETMESHSHAPE_HH_

#include "gazebo/physics/MeshShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class BulletMesh;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_bullet Bullet Physics
    /// \{

    /// \brief Triangle mesh collision
    class GAZEBO_VISIBLE BulletMeshShape : public MeshShape
    {
      /// \brief Constructor
      public: BulletMeshShape(CollisionPtr _parent);

      /// \brief Destructor
      public: virtual ~BulletMeshShape();

      /// \brief Load the trimesh
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the mesh shape.
      protected: virtual void Init();

      /// \brief Bullet collision mesh helper class
      private: BulletMesh *bulletMesh;
    };
    /// \}
  }
}
#endif
