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


#include "OgreCreator.hh"
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
void PlaneGeom::SetAltitude(const Vector3 &pos)
{
  dVector4 vec4;
  dGeomPlaneGetParams(this->geomId, vec4);

  // Compute "altitude": scalar product of position and normal
  vec4[3] = vec4[0] * pos.x + vec4[1] * pos.y + vec4[2] * pos.z;
  dGeomPlaneSetParams(this->geomId, vec4[0], vec4[1], vec4[2], vec4[3]);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneGeom::LoadChild(XMLConfigNode *node)
{
  Vector3 perp;

  double altitude = 0;
  Vector3 normal = node->GetVector3("normal",Vector3(0,0,1));

  OgreCreator::CreatePlane(node,this->GetVisualNode());

  this->SetGeom(dCreatePlane(this->spaceId, normal.x, normal.y, normal.z, altitude),false);

  this->contact->kp = dInfinity;
  this->contact->kd = 0;
  this->contact->mu1 = dInfinity;
  this->contact->mu2 = dInfinity;

}

