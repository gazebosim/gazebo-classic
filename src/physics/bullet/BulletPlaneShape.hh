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
/* Desc: Plane shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id$
 */

#ifndef BULLETPLANESHAPE_HH
#define BULLETPLANESHAPE_HH

#include "common/Exception.hh"
#include "BulletCollision.hh"
#include "BulletPhysics.hh"
#include "PlaneShape.hh"

namespace gazebo
{
	namespace physics
{
  class Link;
  class XMLConfig;

  /// \brief Bullet collision for an infinite plane.
  class BulletPlaneShape : public PlaneShape
  {
    /// \brief Constructor
    public: BulletPlaneShape(Collision *parent) : PlaneShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletPlaneShape() {}
  
    /// \brief Set the altitude of the plane
    public: void SetAltitude(const math::Vector3 &pos)
            {
              PlaneShape::SetAltitude(pos);
            }

    /// \brief Create the plane
    public: void CreatePlane()
            {
              BulletCollision *bParent = (BulletCollision*)(this->parent);
              PlaneShape::CreatePlane();

              btmath::Vector3 vec( (**normalP).x, (**normalP).y, (**normalP).z);

              btCollisionShape *btshape = new btStaticPlaneShape(vec, 0.0);

              bParent->SetCollisionShape(btshape);
            }
  };
  
  /// \}
}
}
}
#endif
