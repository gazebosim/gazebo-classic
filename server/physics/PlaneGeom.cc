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
PlaneGeom::PlaneGeom(Body *body)
    : Geom(body)
{
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
PlaneGeom::~PlaneGeom()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Set the altitude of the plane
void PlaneGeom::SetAltitude(double altitude)
{
  dVector4 vec4;
  dGeomPlaneGetParams(this->geomId, vec4);

  vec4[3] = altitude;
  dGeomPlaneSetParams(this->geomId, vec4[0], vec4[1], vec4[2], vec4[3]);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneGeom::LoadChild(XMLConfigNode *node)
{
  Vector3 perp;

  double altitude = 0;
  Vector3 normal = node->GetVector3("normal",Vector3(0,1,0));
  Vector2<double> size = node->GetVector2d("size",Vector2<double>(1000, 1000));
  Vector2<double> segments = node->GetVector2d("segments",Vector2<double>(10, 10));
  Vector2<double> uvTile = node->GetVector2d("uvTile",Vector2<double>(1, 1));

  normal.Normalize();
  perp = normal.GetPerpendicular();

  // Create an ODE plane geom
  // This geom is not placable
  this->SetGeom(dCreatePlane(this->spaceId, normal.x, normal.y, normal.z, altitude),false);

  Ogre::Plane plane(Ogre::Vector3(normal.y, normal.z, normal.x), 0);

  Ogre::MeshManager::getSingleton().createPlane(this->GetName(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
      size.x, size.y, 
      (int)segments.x, (int)segments.y,
      true,1,
      uvTile.x, uvTile.y, 
      Ogre::Vector3(perp.y, perp.z, perp.x));

  this->AttachMesh(this->GetName());
  this->SetCastShadows(false);

  this->contact->kp = dInfinity;
  this->contact->kd = 0;
  this->contact->mu1 = dInfinity;
  this->contact->mu2 = dInfinity;

}
