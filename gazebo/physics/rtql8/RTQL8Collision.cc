/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: RTQL8Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>

#include "common/Console.hh"
#include "math/Box.hh"

//#include "physics/SurfaceParams.hh"
//#include "physics/rtql8/RTQL8Physics.hh"
//#include "physics/rtql8/RTQL8Link.hh"
#include "physics/rtql8/RTQL8Collision.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Collision::RTQL8Collision(LinkPtr _link)
: Collision(_link)
{
//   this->SetName("ODE_Collision");
//   this->collisionId = NULL;
//   this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeNull;
}

//////////////////////////////////////////////////
RTQL8Collision::~RTQL8Collision()
{
//   if (this->collisionId)
//     dGeomDestroy(this->collisionId);
//   this->collisionId = NULL;
}

//////////////////////////////////////////////////
void RTQL8Collision::Load(sdf::ElementPtr _sdf)
{
//   Collision::Load(_sdf);
// 
//   this->SetSpaceId(
//       boost::shared_static_cast<ODELink>(this->link)->GetSpaceId());
// 
//   if (this->IsStatic())
//   {
//     this->SetCategoryBits(GZ_FIXED_COLLIDE);
//     this->SetCollideBits(~GZ_FIXED_COLLIDE);
//   }
}

//////////////////////////////////////////////////
void RTQL8Collision::Fini()
{
  /*
     if (this->collisionId)
     dGeomDestroy(this->collisionId);
     this->collisionId = NULL;

     if (this->spaceId)
     dSpaceDestroy(this->spaceId);
     this->spaceId = NULL;
     */

  Collision::Fini();
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCollision(bool _placeable)
{
//   // Must go first in this function
//   this->collisionId = _collisionId;
// 
//   Collision::SetCollision(_placeable);
// 
//   if (dGeomGetSpace(this->collisionId) == 0)
//   {
//     dSpaceAdd(this->spaceId, this->collisionId);
//     assert(dGeomGetSpace(this->collisionId) != 0);
//   }
// 
//   if (this->collisionId && this->placeable)
//   {
//     if (this->IsStatic())
//       this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeGlobal;
//     else
//       this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeRelative;
//   }
//   else
//   {
//     this->onPoseChangeFunc = &RTQL8Collision::OnPoseChangeNull;
//   }
// 
//   dGeomSetData(this->collisionId, this);
}

//////////////////////////////////////////////////
void RTQL8Collision::OnPoseChange()
{
//   // Update all the models
//   // (*this.*onPoseChangeFunc)();
// 
//   if (this->IsStatic() && this->collisionId && this->placeable)
//     this->OnPoseChangeGlobal();
//   else if (this->collisionId && this->placeable)
//     this->OnPoseChangeRelative();
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCategoryBits(unsigned int _bits)
{
//   if (this->collisionId)
//     dGeomSetCategoryBits(this->collisionId, _bits);
//   if (this->spaceId)
//     dGeomSetCategoryBits((dGeomID)this->spaceId, _bits);
}

//////////////////////////////////////////////////
void RTQL8Collision::SetCollideBits(unsigned int _bits)
{
//   if (this->collisionId)
//     dGeomSetCollideBits(this->collisionId, _bits);
//   if (this->spaceId)
//     dGeomSetCollideBits((dGeomID)this->spaceId, _bits);
}

//////////////////////////////////////////////////
math::Box RTQL8Collision::GetBoundingBox() const
{
  math::Box box;
//   dReal aabb[6];
// 
//   memset(aabb, 0, 6 * sizeof(dReal));
// 
//   if (this->collisionId == NULL)
//     printf("HOW IS THIS NULL\n");
// 
//   // if (this->collisionId && this->type != Shape::PLANE)
//   dGeomGetAABB(this->collisionId, aabb);
// 
//   box.min.Set(aabb[0], aabb[2], aabb[4]);
//   box.max.Set(aabb[1], aabb[3], aabb[5]);

  return box;
}
