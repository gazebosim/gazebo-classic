/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Plane shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id$
 */

#ifndef BULLETPLANESHAPE_HH
#define BULLETPLANESHAPE_HH

#include "GazeboError.hh"
#include "BulletGeom.hh"
#include "BulletPhysics.hh"
#include "PlaneShape.hh"

namespace gazebo
{
  class Body;
  class XMLConfig;

  /// \brief Bullet geom for an infinite plane.
  class BulletPlaneShape : public PlaneShape
  {
    /// \brief Constructor
    public: BulletPlaneShape(Geom *parent) : PlaneShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletPlaneShape() {}
  
    /// \brief Set the altitude of the plane
    public: void SetAltitude(const Vector3 &pos)
            {
              PlaneShape::SetAltitude(pos);
            }

    /// \brief Create the plane
    public: void CreatePlane()
            {
              BulletGeom *bParent = (BulletGeom*)(this->parent);
              PlaneShape::CreatePlane();

              btVector3 vec( (**normalP).x, (**normalP).y, (**normalP).z);

              btCollisionShape *btshape = new btStaticPlaneShape(vec, 0.0);

              bParent->SetCollisionShape(btshape);
            }
  };
  
  /// \}
}
#endif
