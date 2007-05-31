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
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: RayGeom.cc,v 1.26 2006/02/03 02:53:03 natepak Exp $
 */

#include <assert.h>
#include <float.h>
#include <Ogre.h>
#include <ode/ode.h>

#include "Body.hh"
#include "RayGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
RayGeom::RayGeom( Body *body )
  : Geom( body )
{
  // Create default ray with unit length
  this->SetGeom( dCreateRay( this->spaceId, 1.0 ),  false );

  this->contactDepth = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RayGeom::~RayGeom()
{
}

//////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction
void RayGeom::Set(const Vector3 &pos, const Vector3 &dir)
{
  dGeomRaySet(this->geomId, pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
}


//////////////////////////////////////////////////////////////////////////////
// Get the starting point and direction
void RayGeom::Get(Vector3 &pos, Vector3 &dir)
{
  dVector3 p, d;  
  dGeomRayGet(this->geomId, p, d);

  pos.Set(p[0], p[1], p[2]);
  dir.Set(d[0], d[1], d[2]);
}


//////////////////////////////////////////////////////////////////////////////
// Set the length of the ray
void RayGeom::SetLength( const double len )
{
  dGeomRaySetLength( this->geomId, len );
}


//////////////////////////////////////////////////////////////////////////////
// Get the length of the ray
double RayGeom::GetLength() const
{
  return dGeomRayGetLength( this->geomId );
}
