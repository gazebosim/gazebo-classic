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
  this->normalP = new Param<Vector3>("normal",Vector3(0,0,1),0);
  this->sizeP = new Param<Vector2<double> >("size",
      Vector2<double>(1000, 1000), 0);
  this->segmentsP = new Param<Vector2<double> >("segments",
      Vector2<double>(10, 10), 0);
  this->uvTileP = new Param<Vector2<double> >("uvTile",
      Vector2<double>(1, 1), 0);
  this->materialP = new Param<std::string>("material","",1);
  this->castShadowsP = new Param<bool>("castShadows", false, 0);

}

//////////////////////////////////////////////////////////////////////////////
// Destructor
PlaneGeom::~PlaneGeom()
{
  delete this->normalP;
  delete this->sizeP;
  delete this->segmentsP;
  delete this->uvTileP;
  delete this->materialP;
  delete this->castShadowsP;
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

  this->normalP->Load(node);
  this->sizeP->Load(node);
  this->segmentsP->Load(node);
  this->uvTileP->Load(node);
  this->materialP->Load(node);
  this->castShadowsP->Load(node);

  OgreCreator::CreatePlane(**(this->normalP), **(this->sizeP), 
      **(this->segmentsP), **(this->uvTileP), **(this->materialP),
      **(this->castShadowsP), this->GetVisualNode());

  this->SetGeom(dCreatePlane(this->spaceId, this->normalP->GetValue().x, 
        this->normalP->GetValue().y, this->normalP->GetValue().z, altitude),false);

  this->contact->kp = dInfinity;
  this->contact->kd = 0;
  this->contact->mu1 = dInfinity;
  this->contact->mu2 = dInfinity;
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void PlaneGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->normalP) << "\n";
  stream << prefix << *(this->sizeP) << "\n";
  stream << prefix << *(this->segmentsP) << "\n";
  stream << prefix << *(this->uvTileP) << "\n";
  stream << prefix << *(this->materialP) << "\n";
  stream << prefix << *(this->castShadowsP) << "\n";
}
