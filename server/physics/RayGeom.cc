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
 * CVS: $Id$
 */

#include <assert.h>
#include <float.h>
#include <ode/ode.h>

#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "OgreCreator.hh"
#include "Body.hh"
#include "Global.hh"
#include "RayGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
RayGeom::RayGeom( Body *body, bool displayRays )
    : Geom( body),
    line(NULL)
{
  this->SetName("Ray");

  // Create default ray with unit length
  this->SetGeom( dCreateRay( this->spaceId, 1.0 ),  false );

  if (displayRays)
  {
    this->line = OgreCreator::Instance()->CreateDynamicLine(OgreDynamicRenderable::OT_LINE_LIST);

    // Add two points
    this->line->AddPoint(Vector3(0,0,0));
    this->line->AddPoint(Vector3(0,0,0));

    this->visualNode->AttachObject(this->line);

    this->line->setMaterial("Gazebo/BlueEmissive");
    this->line->setVisibilityFlags(GZ_LASER_CAMERA);
  }

  this->contactLen = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
RayGeom::~RayGeom()
{
  if (this->line)
  {
    delete this->line;
    this->line = NULL;
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the ray geom
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

  if (this->line)
  {
    // Set the line's position relative to it's parent scene node
    this->line->SetPoint(0, this->relativeStartPos);
    this->line->SetPoint(1, this->relativeEndPos);
  }
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

  this->relativeEndPos = dir * len + this->relativeStartPos;

  if (this->line)
  {
    this->line->SetPoint(1,  this->relativeEndPos);
  }
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

//////////////////////////////////////////////////////////////////////////////
/// \brief Load thte ray
void RayGeom::LoadChild(XMLConfigNode * /*node*/)
{
}


