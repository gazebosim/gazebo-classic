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
 * SVN: $Id:$
 */

#include "ODEBody.hh"
#include "ODEGeom.hh"
#include "ODEPhysics.hh"
#include "rendering/Visual.hh"
#include "rendering/OgreDynamicLines.hh"
#include "ODERayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
ODERayShape::ODERayShape( Geom *parent, bool displayRays )
    : RayShape(parent, displayRays)
{
  this->SetName("ODE Ray Shape");

  ODEGeom *geom = (ODEGeom*)this->geomParent;

  // Create default ray with unit length
  geom->SetGeom( dCreateRay( geom->GetSpaceId(), 1.0 ),  false );
}

//////////////////////////////////////////////////////////////////////////////
// Destructor
ODERayShape::~ODERayShape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the ray geom
void ODERayShape::Update()
{
  ODEGeom *geom = (ODEGeom*)this->geomParent;

  Vector3 dir;

  this->globalStartPos = this->geomParent->GetBody()->GetWorldPose().CoordPositionAdd(
      this->relativeStartPos);
  this->globalEndPos = this->geomParent->GetBody()->GetWorldPose().CoordPositionAdd(
      this->relativeEndPos);

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  if (this->contactLen != 0)
  {
    dGeomRaySet(geom->GetGeomId(), this->globalStartPos.x,
        this->globalStartPos.y, this->globalStartPos.z,
        dir.x, dir.y, dir.z);

    dGeomRaySetLength( geom->GetGeomId(),
        this->globalStartPos.Distance(this->globalEndPos) );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the starting point and direction
void ODERayShape::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{
  Vector3 dir;
  ODEGeom *geom = (ODEGeom*)this->geomParent;

  RayShape::SetPoints(posStart, posEnd);

  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

  dGeomRaySet(geom->GetGeomId(), this->globalStartPos.x,
              this->globalStartPos.y, this->globalStartPos.z,
              dir.x, dir.y, dir.z);

  dGeomRaySetLength( geom->GetGeomId(),
                     this->globalStartPos.Distance(this->globalEndPos) );
}
