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
/* Desc: Cylinder geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <Ogre.h>

#include "Body.hh"
#include "CylinderGeom.hh"
#include "dCylinder.h" // For declarations missing from ODE

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
CylinderGeom::CylinderGeom(Body *body, double radius, double length, const std::string &meshName )
    : Geom(body)
{
  // Initialize mass matrix
  dMassSetCylinder(&this->mass, 1.0, 2, radius, length);
  
  this->SetGeom( dCreateCCylinder( this->body->spaceId, radius, length ), true );

  // ODE cylinders are oriented along the y-axis; fix this
  // so the are along the z-axis
  //this->SetExtraRotation(GzQuaternFromAxis(1, 0, 0, M_PI / 2));
  //this->SetRelativeRotation(GzQuaternIdent());
 
  // Get the mesh
  this->AttachMesh(meshName);

  // Set the size of the cylinder
  this->ScaleMesh(Vector3(radius,radius,length));

  // Set the default position of the cylinder
  //this->SetMeshPosition(GzVectorSet(0,length,0));

  // Allow it to cast shadows
  this->SetCastShadows(true);

  return;
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
CylinderGeom::~CylinderGeom()
{
  return;
}
