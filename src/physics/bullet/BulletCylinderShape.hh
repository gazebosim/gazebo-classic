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
/* Desc: Cylinder shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id$
 */

#ifndef BULLETCYLINDERSHAPE_HH
#define BULLETCYLINDERSHAPE_HH

#include "common/GazeboError.hh"
#include "BulletGeom.hh"
#include "BulletPhysics.hh"
#include "CylinderShape.hh"

namespace gazebo
{
	namespace physics
{
  /// \brief Cylinder geom
  class BulletCylinderShape : public CylinderShape
  {
    /// \brief Constructor
    public: BulletCylinderShape(Geom *parent) : CylinderShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletCylinderShape() {}

    /// \brief Set the size of the cylinder
    public: void SetSize( const Vector2d &size )
            {
              CylinderShape::SetSize(size);
              BulletGeom *bParent = (BulletGeom*)(this->parent);
  
              bParent->SetCollisionShape( new btCylinderShapeZ(
                  btcommon::Vector3(size.x * 0.5, size.x*0.5, size.y*0.5)) );
            }
  };

  /// \}
}
}
}
#endif
