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
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <ode/ode.h>
#include <Ogre.h>

#include "Body.hh"
#include "SphereGeom.hh"


//////////////////////////////////////////////////////////////////////////////
// Constructor
SphereGeom::SphereGeom(Body *body, double radius, const std::string &meshName )
    : Geom(body)
{
  // Initialize box mass matrix
  dMassSetSphere(&this->mass, 1, radius);

  // Create the sphere geometry
  this->SetGeom(dCreateSphere( this->body->spaceId, radius ), true);

  // Get the sphere mesh
  this->AttachMesh(meshName);

  // Set the size of the sphere
  this->ScaleMesh(Vector3(radius,radius,radius));

  // Allow the sphere to cast shadows
  this->SetCastShadows(true);
   
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SphereGeom::~SphereGeom()
{
  return;
}
