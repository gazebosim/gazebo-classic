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
/* Desc: Cylinder shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id$
 */

#ifndef BULLETCYLINDERSHAPE_HH
#define BULLETCYLINDERSHAPE_HH

#include "GazeboError.hh"
#include "BulletGeom.hh"
#include "BulletPhysics.hh"
#include "CylinderShape.hh"

namespace gazebo
{
  /// \brief Cylinder geom
  class BulletCylinderShape : public CylinderShape
  {
    /// \brief Constructor
    public: BulletCylinderShape(Geom *parent) : CylinderShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletCylinderShape() {}

    /// \brief Set the size of the cylinder
    public: void SetSize( const Vector2<double> &size )
            {
              CylinderShape::SetSize(size);
              BulletGeom *bParent = (BulletGeom*)(this->parent);
  
              bParent->SetCollisionShape( new btCylinderShapeZ(
                  btVector3(size.x * 0.5, size.x*0.5, size.y*0.5)) );
            }
  };

  /// \}
}
#endif
