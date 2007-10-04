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

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
CylinderGeom::CylinderGeom(Body *body)
  : Geom(body)
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
CylinderGeom::~CylinderGeom()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the cylinder
void CylinderGeom::LoadChild(XMLConfigNode *node)
{
  double radius = node->GetTupleDouble("size",0,1.0);
  double length = node->GetTupleDouble("size",1,1.0);

  // Initialize mass matrix
  dMassSetCylinderTotal(&this->mass, this->dblMass, 3, radius, length);
  
  this->SetGeom( dCreateCylinder( 0, radius, length ), true );

  // Get the mesh
  if (this->meshName.empty() || this->meshName == "default")
    this->AttachMesh("unit_cylinder");
  else
    this->AttachMesh(this->meshName);

  Ogre::Vector3 meshSize = this->ogreObj->getBoundingBox().getSize();

  radius /= meshSize.x;
  length /= meshSize.z;

  // Set the size of the cylinder
  this->ScaleMesh(Vector3(radius*2,radius*2,length));

  // Allow it to cast shadows
  this->SetCastShadows(true);
}
