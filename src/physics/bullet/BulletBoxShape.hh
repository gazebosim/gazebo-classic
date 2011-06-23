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
/* Desc: Box shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef BULLETBOXSHAPE_HH
#define BULLETBOXSHAPE_HH

#include "common/Exception.hh"
#include "BulletGeom.hh"
#include "BulletPhysics.hh"
#include "BoxShape.hh"

namespace gazebo
{
	namespace physics
{
  /// \brief Box geom
  class BulletBoxShape : public BoxShape
  {
    /// \brief Constructor
    public: BulletBoxShape(Geom *parent) : BoxShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletBoxShape() {}

    /// \brief Set the size of the box
    public: void SetSize( const math::Vector3 &size )
            {
              BoxShape::SetSize(size);
              BulletGeom *bParent = (BulletGeom*)(this->parent);

              /// Bullet requires the half-extents of the box 
              bParent->SetCollisionShape( new btBoxShape(
                  btmath::Vector3(size.x*0.5, size.y*0.5, size.z*0.5)) );
            }
  };

}
}
}
#endif
