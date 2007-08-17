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
  : Geom( body,"Ray" )
{
  // Create default ray with unit length
  this->SetGeom( dCreateRay( this->spaceId, 1.0 ),  false );
 
  this->line = new OgreDynamicLines(Ogre::RenderOperation::OT_LINE_LIST);

  // Add two points
  this->line->AddPoint(Vector3(0,0,0));
  this->line->AddPoint(Vector3(0,0,0));

  this->AttachObject(this->line);

  this->line->setMaterial("Gazebo/BlueLaser");

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

void RayGeom::Update()
{
  Vector3 dir;

  this->globalStartPos = this->body->GetPose().CoordPositionAdd(this->relativeStartPos);
  this->globalEndPos = this->body->GetPose().CoordPositionAdd(this->relativeEndPos);

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  dGeomRaySet(this->geomId, this->globalStartPos.x, 
      this->globalStartPos.y, this->globalStartPos.z, 
      dir.x, dir.y, dir.z);

  dGeomRaySetLength( this->geomId, 
      this->globalStartPos.Distance(this->globalEndPos) );
}

//////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction 
void RayGeom::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{
  Vector3 dir;

  this->relativeStartPos = posStart;
  this->relativeEndPos = posEnd;

  this->globalStartPos = this->body->GetPose().CoordPositionAdd(this->relativeStartPos);
  this->globalEndPos = this->body->GetPose().CoordPositionAdd(this->relativeEndPos);

  // Compute the direction of the ray
  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  dGeomRaySet(this->geomId, this->globalStartPos.x, 
              this->globalStartPos.y, this->globalStartPos.z, 
              dir.x, dir.y, dir.z);

  dGeomRaySetLength( this->geomId, 
      this->globalStartPos.Distance(this->globalEndPos) );

  // Set the line's position relative to it's parent scene node
  this->line->SetPoint(0, this->relativeStartPos);
  this->line->SetPoint(1, this->relativeEndPos);
  this->line->Update();
}

//////////////////////////////////////////////////////////////////////////////
// Get the relative starting and ending points
void RayGeom::GetRelativePoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->relativeStartPos;
  posB = this->relativeEndPos;
}

//////////////////////////////////////////////////////////////////////////////
// Get the global starting and ending points
void RayGeom::GetGlobalPoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->globalStartPos;
  posB = this->globalEndPos;
}

//////////////////////////////////////////////////////////////////////////////
// Set the length of the ray
void RayGeom::SetLength( const double len )
{
  //dGeomRaySetLength( this->geomId, len );
  this->contactLen=len;

  Vector3 dir = this->relativeEndPos - this->relativeStartPos;
  dir.Normalize();

  this->line->SetPoint(1,  dir * len + this->relativeStartPos);
  this->line->Update();
}


//////////////////////////////////////////////////////////////////////////////
// Get the length of the ray
double RayGeom::GetLength() const
{
  return this->contactLen;
  //return dGeomRayGetLength( this->geomId );
}


//////////////////////////////////////////////////////////////////////////////
/// Set the retro-reflectivness detected by this ray
void RayGeom::SetRetro( float retro )
{
  this->contactRetro = retro;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the retro-reflectivness detected by this ray
float RayGeom::GetRetro() const
{
  return this->contactRetro;
}

//////////////////////////////////////////////////////////////////////////////
/// Set the fiducial id detected by this ray
void RayGeom::SetFiducial( int fid )
{
  this->contactFiducial = fid;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the fiducial id detected by this ray
int RayGeom::GetFiducial() const
{
  return this->contactFiducial;
}
