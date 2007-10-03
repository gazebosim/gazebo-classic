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
/* Desc: Box geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <Ogre.h>
#include <ode/ode.h>

#include "Body.hh"
#include "BoxGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
/*BoxGeom::BoxGeom(Body *body, const std::string &name, Vector3 size, double mass, const std::string &meshName )
    : Geom(body,name)
    */
BoxGeom::BoxGeom(Body *body)
    : Geom(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BoxGeom::~BoxGeom()
{
}
//////////////////////////////////////////////////////////////////////////////
/// Load the box
void BoxGeom::LoadChild(XMLConfigNode *node)
{
  Vector3 size = node->GetVector3("size",Vector3(1,1,1));

  // Initialize box mass matrix
  dMassSetBoxTotal(&this->mass, this->dblMass, size.x, size.y, size.z);

  // Create a box geometry with box mass matrix
  this->SetGeom(dCreateBox( 0, size.x, size.y, size.z), true );

  // Get the box mesh
  if (this->meshName.empty() || this->meshName == "default")
  {
    this->AttachMesh("unit_box");

    // Set the size of the box
    this->ScaleMesh(size);
  }
  else
    this->AttachMesh(this->meshName);

  // Allow the box to cast shadows
  this->SetCastShadows(true);
}
