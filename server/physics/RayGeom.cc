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

#include "OgreDynamicLines.hh"
#include "Body.hh"
#include "RayGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
RayGeom::RayGeom( Body *body )
  : Geom( body )
{
  body->SetEnabled(false);

  // Create default ray with unit length
  this->SetGeom( dCreateRay( this->spaceId, 1.0 ),  false );
 
  this->line = new OgreDynamicLines(Ogre::RenderOperation::OT_LINE_LIST);

  // Add two points
  this->line->AddPoint(Vector3(0,0,0));
  this->line->AddPoint(Vector3(0,0,0));

  this->AttachObject(line);

  this->contactLen = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RayGeom::~RayGeom()
{
  delete this->line;
  this->line = NULL;
}

//////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction, in global cs
void RayGeom::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{

  this->pos= posStart;
  this->dir = posEnd - this->pos;
  this->dir.Normalize();

  dGeomRaySet(this->geomId, this->pos.x, this->pos.y, this->pos.z, 
              this->dir.x, this->dir.y, this->dir.z);

  dGeomRaySetLength( this->geomId, posEnd.Distance(this->pos) );

  // Get the gobal position of the scene node
  /*Ogre::Vector3 olinePos = this->sceneNode->_getDerivedPosition();
  Vector3 linePos;

  linePos.x = olinePos.x;
  linePos.y = olinePos.y;
  linePos.z = olinePos.z;

  // Set the line's position relative to it's parent scene node
  this->line->SetPoint(0, this->pos-linePos);
  this->line->SetPoint(1, posEnd-linePos);
  this->line->Update();
  */
  
}

void RayGeom::Set(const Vector3 &posStart, const Vector3 &dir, double length)
{
  Vector3 end;
  this->dir = dir;
  this->pos = posStart;

  dGeomRaySet(this->geomId, this->pos.x, this->pos.y, this->pos.z, 
              this->dir.x, this->dir.y, this->dir.z);

  dGeomRaySetLength( this->geomId, length );

  end = this->pos + this->dir*length;

  // Get the gobal position of the scene node
  /*Ogre::Vector3 olinePos = this->sceneNode->_getDerivedPosition();
  Vector3 linePos;

  linePos.x = olinePos.x;
  linePos.y = olinePos.y;
  linePos.z = olinePos.z;

  // Set the line's position relative to it's parent scene node
  this->line->SetPoint(0, this->pos - linePos);
  this->line->SetPoint(1, end - linePos);
  this->line->Update();
  */
}

//////////////////////////////////////////////////////////////////////////////
// Get the starting and ending point, in global cs
void RayGeom::GetPoints(Vector3 &posA, Vector3 &posB)
{
  dVector3 p, d;  
  Vector3 dir;
  dGeomRayGet(this->geomId, p, d);
  
  this->pos.Set(p[0], p[1], p[2]);
  this->dir.Set(d[0], d[1], d[2]);

  posA = this->pos;
  posB = this->pos+(this->dir*this->GetLength());
}

//////////////////////////////////////////////////////////////////////////////
// Get the starting point and direction
void RayGeom::Get(Vector3 &position, Vector3 &direction)
{
  dVector3 p, d;  
  Vector3 dir;
  dGeomRayGet(this->geomId, p, d);
  
  this->pos.Set(p[0], p[1], p[2]);
  this->dir.Set(d[0], d[1], d[2]);

  position = this->pos;
  direction = this->dir;
}

//////////////////////////////////////////////////////////////////////////////
// Set the length of the ray
void RayGeom::SetLength( const double len )
{
  dGeomRaySetLength( this->geomId, len );

  Vector3 startPt = this->line->GetPoint(0);

  this->line->SetPoint(1,  this->dir*len+startPt);
  this->line->Update();
}


//////////////////////////////////////////////////////////////////////////////
// Get the length of the ray
double RayGeom::GetLength() const
{
  return dGeomRayGetLength( this->geomId );
}
