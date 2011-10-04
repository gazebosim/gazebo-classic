/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: A ray shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 */

#include <vector>
#include <list>
#include <string>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include "transport/TransportTypes.hh"

#include "common/CommonTypes.hh"
#include "common/Event.hh"
#include "msgs/msgs.h"

#include "physics/PhysicsTypes.hh"
#include "sdf/sdf.h"

#include "physics/Collision.hh"
#include "physics/RayShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
RayShape::RayShape( CollisionPtr _parent, bool /*_displayRays*/ ) 
  : Shape(_parent)
{
  this->AddType(RAY_SHAPE);
  this->SetName("Ray");

  /*this->vis_pub = transport::advertise<msgs::Visual>("~/visual");
  if (displayRays)
  {
    msgs::Visual msg;
    msg.SetName( this->GetName() );
    msg.set_parent_id( this->collisionParent->GetName() );
    msg.set_render_type( msgs::Visual::LINE_LIST );

    msg.set_material( "Gazebo/BlueGlow" );
    this->vis_pub->Publish(msg);

    // NATY: put back in
    //this->lineMsg->visibility = GZ_LASER_CAMERA;
  }*/

  this->contactLen = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;

  this->collisionParent->SetSaveable(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RayShape::~RayShape()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set to true in order to view individual rays
void RayShape::SetDisplayType( bool /*_displayRays*/ )
{
  /* NATY: do we need this?
  if (Simulator::Instance()->GetRenderEngineEnabled() )
  {
    if (!displayRays)
      this->collisionParent->GetVisualNode()->DetachObjects();
    else
      this->collisionParent->GetVisualNode()->AttachObject(this->line);
  }
  */
}
 
////////////////////////////////////////////////////////////////////////////////
/// Set the ray based on starting and ending points relative to the body
void RayShape::SetPoints(const math::Vector3 &posStart, const math::Vector3 &posEnd)
{
  math::Vector3 dir;

  this->relativeStartPos = posStart;
  this->relativeEndPos = posEnd;

  this->globalStartPos = this->collisionParent->GetWorldPose().CoordPositionAdd(
      this->relativeStartPos);
  this->globalEndPos = this->collisionParent->GetWorldPose().CoordPositionAdd(
      this->relativeEndPos);

  // Compute the direction of the ray
  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  msgs::Visual msg;
  msg.set_name( this->GetName() );

  msgs::Vector3d *pt = msg.mutable_geometry()->add_points(); 
  msgs::Set( pt,  this->relativeStartPos );
  pt = msg.mutable_geometry()->add_points();
  msgs::Set(pt, this->relativeEndPos );

  //this->vis_pub->Publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the relative starting and ending points
void RayShape::GetRelativePoints(math::Vector3 &posA, math::Vector3 &posB)
{
  posA = this->relativeStartPos;
  posB = this->relativeEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the global starting and ending points
void RayShape::GetGlobalPoints(math::Vector3 &posA, math::Vector3 &posB)
{
  posA = this->globalStartPos;
  posB = this->globalEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the length of the ray
void RayShape::SetLength( double len )
{
  this->contactLen=len;

  math::Vector3 dir = this->relativeEndPos - this->relativeStartPos;
  dir.Normalize();

  this->relativeEndPos = dir * len + this->relativeStartPos;


  msgs::Visual msg;
  msg.set_name( this->GetName() );

  msgs::Vector3d *pt = msg.mutable_geometry()->add_points(); 
  msgs::Set(pt, this->relativeStartPos );
  pt = msg.mutable_geometry()->add_points();
  msgs::Set(pt,  this->relativeEndPos );
  //this->vis_pub->Publish(msg);
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
void RayShape::Load( sdf::ElementPtr &_sdf ) 
{
  Shape::Load(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// In the ray
void RayShape::Init()
{
}
