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
/* Desc: A ray shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#include "Messages.hh"
#include "Simulator.hh"
#include "RayShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
RayShape::RayShape( Geom *parent, bool displayRays ) : Shape(parent)
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  if (displayRays && Simulator::Instance()->GetRenderEngineEnabled() )
  {
    this->lineMsg = new VisualMsg();
    this->lineMsg->id = this->GetName();
    this->lineMsg->parentId = this->geomParent->GetName();
    this->lineMsg->render = VisualMsg::LINE_LIST;

    // Add two points
    this->lineMsg->points.push_back(Vector3(0,0,0));
    this->lineMsg->points.push_back(Vector3(0,0,0));

    this->lineMsg->material = "Gazebo/BlueGlow";

    // NATY: put back in
    //this->lineMsg->visibility = GZ_LASER_CAMERA;
  }

  this->contactLen = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;

  this->geomParent->SetSaveable(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RayShape::~RayShape()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true in order to view individual rays
void RayShape::SetDisplayType( bool displayRays )
{
  /* NATY: do we need this?
  if (Simulator::Instance()->GetRenderEngineEnabled() )
  {
    if (!displayRays)
      this->geomParent->GetVisualNode()->DetachObjects();
    else
      this->geomParent->GetVisualNode()->AttachObject(this->line);
  }
  */
}
 
////////////////////////////////////////////////////////////////////////////////
/// Set the ray based on starting and ending points relative to the body
void RayShape::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{
  Vector3 dir;

  this->relativeStartPos = posStart;
  this->relativeEndPos = posEnd;

  this->globalStartPos = this->geomParent->GetWorldPose().CoordPositionAdd(
      this->relativeStartPos);
  this->globalEndPos = this->geomParent->GetWorldPose().CoordPositionAdd(
      this->relativeEndPos);

  // Compute the direction of the ray
  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  // Set the line's position relative to it's parent scene node
  this->lineMsg->points[0] = this->relativeStartPos;
  this->lineMsg->points[1] = this->relativeEndPos;
  Simulator::Instance()->SendMessage( *this->lineMsg );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the relative starting and ending points
void RayShape::GetRelativePoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->relativeStartPos;
  posB = this->relativeEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the global starting and ending points
void RayShape::GetGlobalPoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->globalStartPos;
  posB = this->globalEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the length of the ray
void RayShape::SetLength( double len )
{
  this->contactLen=len;

  Vector3 dir = this->relativeEndPos - this->relativeStartPos;
  dir.Normalize();

  this->relativeEndPos = dir * len + this->relativeStartPos;

  this->lineMsg->points[1] = this->relativeEndPos;
  Simulator::Instance()->SendMessage( *this->lineMsg );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the length of the ray
double RayShape::GetLength() const
{
  return this->contactLen;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the retro-reflectivness detected by this ray
void RayShape::SetRetro( float retro )
{
  this->contactRetro = retro;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the retro-reflectivness detected by this ray
float RayShape::GetRetro() const
{
  return this->contactRetro;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the fiducial id detected by this ray
void RayShape::SetFiducial( int fid )
{
  this->contactFiducial = fid;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the fiducial id detected by this ray
int RayShape::GetFiducial() const
{
  return this->contactFiducial;
}

////////////////////////////////////////////////////////////////////////////////
/// Load thte ray
void RayShape::Load(XMLConfigNode *node) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void RayShape::Save(std::string &, std::ostream &) 
{
}
