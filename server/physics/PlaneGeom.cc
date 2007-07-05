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
/* Desc: Infinite plane geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 *
 * Notes: This is not a placeable geometry, so position and
 * orientation are undefined.
 */

#include <math.h>
#include <Ogre.h>

#include "Body.hh"
#include "ContactParams.hh"
#include "PlaneGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
PlaneGeom::PlaneGeom(Body *body, double altitude, Vector3 normal)
    : Geom(body)
{
  Vector3 planeNx, planeNy, planeNz;
  double patchSize;

  // Set the z unit vector (plane normal)
  planeNy = normal;
  planeNy.Normalize();

  // Compute x unit vector (orthogonal to Nz and y)
  planeNx = planeNy.GetCrossProd(Vector3(0.0, 0.0, 1.0));
  planeNx.Normalize();

  // Compute y unit vector (orthogonal to other two vectors)
  planeNz = planeNx.GetCrossProd(planeNy);

  // Size of each renderable planar patch
  patchSize = 2.0;

  // Create an ODE plane geom
  // This geom is not placable
  this->SetGeom(dCreatePlane(this->body->spaceId, 0, 1, 0, 0),false);

  Ogre::Plane plane(Ogre::Vector3(0, 1, 0), 0);

  Ogre::MeshManager::getSingleton().createPlane(this->GetName(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
      1500,1500,200,200,true,10,500,500, Ogre::Vector3(0,0,1));

  this->AttachMesh(this->GetName());
  this->SetCastShadows(false);

  this->contact->kp = dInfinity;
  this->contact->kd = 0;
  this->contact->mu1 = dInfinity;
  this->contact->mu2 = dInfinity;
 
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
PlaneGeom::~PlaneGeom()
{
}
