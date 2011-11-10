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
/* Desc: A ray
 * Author: Nate Koenig
 * Date: 14 Oct 2009
 */

#include "physics/Link.hh"
#include "physics/ode/ODETypes.hh"
#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODERayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODERayShape::ODERayShape( CollisionPtr parent, bool displayRays )
    : RayShape(parent, displayRays)
{
  this->SetName("ODE Ray Shape");

  ODECollisionPtr collision = boost::shared_static_cast<ODECollision>(this->collisionParent);

  // Create default ray with unit length
  collision->SetCollision(dCreateRay( collision->GetSpaceId(), 1.0 ),  false);
  collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
  collision->SetCollideBits(~GZ_SENSOR_COLLIDE);
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODERayShape::~ODERayShape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the ray collision
void ODERayShape::Update()
{
  ODECollisionPtr collision = boost::shared_static_cast<ODECollision>(this->collisionParent);

  math::Vector3 dir;

  this->globalStartPos = this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
      this->relativeStartPos);

  this->globalEndPos = this->collisionParent->GetLink()->GetWorldPose().CoordPositionAdd(
      this->relativeEndPos);

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  //gzerr << "contactLen[" << this->contactLen << "]";
  if (this->contactLen != 0)
  {
    dGeomRaySet(collision->GetCollisionId(), 
        this->globalStartPos.x, this->globalStartPos.y, this->globalStartPos.z,
        dir.x, dir.y, dir.z);

    dGeomRaySetLength( collision->GetCollisionId(),
        this->globalStartPos.Distance(this->globalEndPos) );

     //gzerr << "  linkparent[" << this->collisionParent->GetLink()->GetName() << "]"
     //      << "  linkpose[" << this->collisionParent->GetLink()->GetWorldPose() << "]"
     //      << "  collparent[" << this->collisionParent->GetName() << "]"
     //      << "  collpose[" << this->collisionParent->GetWorldPose() << "]"

     //      << "  relativeStart[" << this->relativeStartPos << "]"
     //      << "  relativeEnd[" << this->relativeEndPos << "]"
     //      << "  globalStartPos[" << this->globalStartPos << "]"
     //      << "  globalEndPos[" << this->globalEndPos << "]"
     //      << "  dir[" << dir << "]"
     //      << "  dist[" << this->globalStartPos.Distance(this->globalEndPos) << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction
void ODERayShape::SetPoints(const math::Vector3 &posStart, const math::Vector3 &posEnd)
{
  math::Vector3 dir;
  ODECollisionPtr collision = boost::shared_static_cast<ODECollision>(this->collisionParent);

  RayShape::SetPoints(posStart, posEnd);

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  dGeomRaySet(collision->GetCollisionId(), this->globalStartPos.x,
              this->globalStartPos.y, this->globalStartPos.z,
              dir.x, dir.y, dir.z);

  dGeomRaySetLength( collision->GetCollisionId(),
                     this->globalStartPos.Distance(this->globalEndPos) );
}
