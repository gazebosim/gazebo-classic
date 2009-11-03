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
/* Desc: Sphere geometry
 * Author: Nate Keonig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#ifndef BULLETSPHERESHAPE_HH
#define BULLETSPHERESHAPE_HH

#include "GazeboError.hh"
#include "BulletPhysics.hh"
#include "SphereShape.hh"

namespace gazebo
{
  /// \brief Bullet sphere geom
  class BulletSphereShape : public SphereShape
  {
    /// \brief Constructor
    public: BulletSphereShape(Geom *parent) : SphereShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletSphereShape() {}

    /// \brief Set the size
    public: void SetSize(const double &radius)
            {
              SphereShape::SetSize(radius);
              BulletGeom *bParent = (BulletGeom*)(this->parent);
              bParent->SetCollisionShape( new btSphereShape(radius) );
            }
  };
}

#endif
