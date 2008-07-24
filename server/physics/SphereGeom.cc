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

#include "OgreVisual.hh"
#include "Body.hh"
#include "SphereGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
SphereGeom::SphereGeom(Body *body)
    : Geom(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SphereGeom::~SphereGeom()
{
}

//////////////////////////////////////////////////////////////////////////////
///  Load the sphere
void SphereGeom::LoadChild(XMLConfigNode *node)
{
  double radius = node->GetDouble("size",1.0,0);

  // Initialize box mass matrix
  dMassSetSphereTotal(&this->mass, this->dblMass, radius);

  // Create the sphere geometry
  this->SetGeom( dCreateSphere(0, radius ), true);

  //to be able to show physics
  /*  this->visualNode->AttachMesh("unit_sphere"); // unit_sphere radius=1 diameter=2
    this->visualNode->SetScale(Vector3(radius, radius ,radius));
    this->visualNode->SetMaterial("Gazebo/GreenEmissive");
    */

}
