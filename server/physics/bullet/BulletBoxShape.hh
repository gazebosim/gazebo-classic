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
/* Desc: Box shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef BULLETBOXSHAPE_HH
#define BULLETBOXSHAPE_HH

#include "GazeboError.hh"
#include "BulletGeom.hh"
#include "BulletPhysics.hh"
#include "BoxShape.hh"

namespace gazebo
{
  /// \brief Box geom
  class BulletBoxShape : public BoxShape
  {
    /// \brief Constructor
    public: BulletBoxShape(Geom *parent) : BoxShape(parent) {}

    /// \brief Destructor
    public: virtual ~BulletBoxShape() {}

    /// \brief Set the size of the box
    public: void SetSize( const Vector3 &size )
            {
              BoxShape::SetSize(size);
              BulletGeom *bParent = (BulletGeom*)(this->parent);

              /// Bullet requires the half-extents of the box 
              bParent->SetCollisionShape( new btBoxShape(
                  btVector3(size.x*0.5, size.y*0.5, size.z*0.5)) );
            }
  };

}
#endif
