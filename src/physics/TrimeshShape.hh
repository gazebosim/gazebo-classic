/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Trimesh geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef TRIMESHSHAPE_HH
#define TRIMESHSHAPE_HH

#include "common/CommonTypes.hh"
#include "physics/PhysicsTypes.hh"

#include "physics/Shape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Triangle mesh collision
    class TrimeshShape : public Shape
    {
      /// \brief Constructor
      public: TrimeshShape(CollisionPtr parent);
  
      /// \brief Destructor
      public: virtual ~TrimeshShape();
  
      public: virtual void Update() {}
  
      /// \brief Load the trimesh
      public: virtual void Load( sdf::ElementPtr &_sdf );

      /// \brief Init the trimesh shape
      public: virtual void Init();

      public: virtual math::Vector3 GetSize() const;

      /// \brief Get the filename of the mesh data
      public: std::string GetFilename() const;
  
      protected: const common::Mesh *mesh;
    };
    /// \}
  }
}
#endif
