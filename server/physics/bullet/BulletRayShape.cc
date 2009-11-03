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
/* Desc: A ray
 * Author: Nate Keonig
 * Date: 24 May 2009
 * SVN: $Id:$
 */

#include "Body.hh"
#include "BulletRayGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletRayGeom::BulletRayGeom( Body *body, bool displayRays )
    : RayGeom<BulletGeom>(body, displayRays)
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletRayGeom::~BulletRayGeom()
{
}
 
//////////////////////////////////////////////////////////////////////////////
// Update the ray geom
void BulletRayGeom::Update()
{
  RayGeom<BulletGeom>::Update();
}

//////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction
void BulletRayGeom::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{
  RayGeom<BulletGeom>::SetPoints(posStart, posEnd);
}
