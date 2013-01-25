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

#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/rtql8/RTQL8Types.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"
#include "gazebo/physics/rtql8/RTQL8Collision.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8RayShape.hh"
#include "gazebo/physics/rtql8/RTQL8MultiRayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
RTQL8MultiRayShape::RTQL8MultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent)
{
//   this->SetName("RTQL8 Multiray Shape");
// 
//   // Create a space to contain the ray space
//   this->superSpaceId = dSimpleSpaceCreate(0);
// 
//   // Create a space to contain all the rays
//   this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);
// 
//   // Set collision bits
//   dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
//   dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);
// 
//   // These three lines may be unessecary
//   RTQL8LinkPtr pLink =
//     boost::shared_static_cast<RTQL8Link>(this->collisionParent->GetLink());
//   pLink->SetSpaceId(this->raySpaceId);
//   boost::shared_static_cast<RTQL8Collision>(_parent)->SetSpaceId(
//       this->raySpaceId);
}

//////////////////////////////////////////////////
RTQL8MultiRayShape::~RTQL8MultiRayShape()
{
//   dSpaceSetCleanup(this->raySpaceId, 0);
//   dSpaceDestroy(this->raySpaceId);
// 
//   dSpaceSetCleanup(this->superSpaceId, 0);
//   dSpaceDestroy(this->superSpaceId);
}

//////////////////////////////////////////////////
void RTQL8MultiRayShape::UpdateRays()
{
//   RTQL8PhysicsPtr ode = boost::shared_dynamic_cast<RTQL8Physics>(
//       this->GetWorld()->GetPhysicsEngine());
// 
//   if (ode == NULL)
//     gzthrow("Invalid physics engine. Must use RTQL8.");
// 
//   // FIXME: Do we need to lock the physics engine here? YES!
//   //        especially when spawning models with sensors
// 
//   ode->GetPhysicsUpdateMutex()->lock();
//   // Do collision detection
//   dSpaceCollide2((dGeomID) (this->superSpaceId),
//       (dGeomID) (ode->GetSpaceId()),
//       this, &UpdateCallback);
//   ode->GetPhysicsUpdateMutex()->unlock();
}

// //////////////////////////////////////////////////
// void RTQL8MultiRayShape::UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2)
// {
//   dContactGeom contact;
//   RTQL8Collision *collision1, *collision2 = NULL;
//   RTQL8Collision *rayCollision = NULL;
//   RTQL8Collision *hitCollision = NULL;
//   RTQL8MultiRayShape *self = NULL;
// 
//   self = static_cast<RTQL8MultiRayShape*>(_data);
// 
//   // Check space
//   if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
//   {
//     if (dGeomGetSpace(_o1) == self->superSpaceId ||
//         dGeomGetSpace(_o2) == self->superSpaceId)
//       dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
// 
//     if (dGeomGetSpace(_o1) == self->raySpaceId ||
//         dGeomGetSpace(_o2) == self->raySpaceId)
//       dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
//   }
//   else
//   {
//     collision1 = NULL;
//     collision2 = NULL;
// 
//     // Get pointers to the underlying collisions
//     if (dGeomGetClass(_o1) == dGeomTransformClass)
//     {
//       collision1 = static_cast<RTQL8Collision*>(
//           dGeomGetData(dGeomTransformGetGeom(_o1)));
//     }
//     else
//       collision1 = static_cast<RTQL8Collision*>(dGeomGetData(_o1));
// 
//     if (dGeomGetClass(_o2) == dGeomTransformClass)
//     {
//       collision2 =
//         static_cast<RTQL8Collision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
//     }
//     else
//     {
//       collision2 = static_cast<RTQL8Collision*>(dGeomGetData(_o2));
//     }
// 
//     assert(collision1 && collision2);
// 
//     rayCollision = NULL;
//     hitCollision = NULL;
// 
//     // Figure out which one is a ray; note that this assumes
//     // that the RTQL8 dRayClass is used *soley* by the RayCollision.
//     if (dGeomGetClass(_o1) == dRayClass)
//     {
//       rayCollision = static_cast<RTQL8Collision*>(collision1);
//       hitCollision = static_cast<RTQL8Collision*>(collision2);
//       dGeomRaySetParams(_o1, 0, 0);
//       dGeomRaySetClosestHit(_o1, 1);
//     }
//     else if (dGeomGetClass(_o2) == dRayClass)
//     {
//       assert(rayCollision == NULL);
//       rayCollision = static_cast<RTQL8Collision*>(collision2);
//       hitCollision = static_cast<RTQL8Collision*>(collision1);
//       dGeomRaySetParams(_o2, 0, 0);
//       dGeomRaySetClosestHit(_o2, 1);
//     }
// 
//     // Check for ray/collision intersections
//     if (rayCollision && hitCollision)
//     {
//       int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));
// 
//       if (n > 0)
//       {
//         RayShapePtr shape = boost::shared_static_cast<RayShape>(
//             rayCollision->GetShape());
//         if (contact.depth < shape->GetLength())
//         {
//           // gzerr << "RTQL8MultiRayShape UpdateCallback dSpaceCollide2 "
//           //      << " depth[" << contact.depth << "]"
//           //      << " position[" << contact.pos[0]
//           //        << ", " << contact.pos[1]
//           //        << ", " << contact.pos[2]
//           //        << ", " << "]"
//           //      << " ray[" << rayCollision->GetName() << "]"
//           //      << " pose[" << rayCollision->GetWorldPose() << "]"
//           //      << " hit[" << hitCollision->GetName() << "]"
//           //      << " pose[" << hitCollision->GetWorldPose() << "]"
//           //      << "\n";
//           shape->SetLength(contact.depth);
//           shape->SetRetro(hitCollision->GetLaserRetro());
//         }
//       }
//     }
//   }
// }
// 
// //////////////////////////////////////////////////
// void RTQL8MultiRayShape::AddRay(const math::Vector3 &_start,
//     const math::Vector3 &_end)
// {
//   MultiRayShape::AddRay(_start, _end);
// 
//   RTQL8CollisionPtr odeCollision(new RTQL8Collision(
//         this->collisionParent->GetLink()));
//   odeCollision->SetName("ode_ray_collision");
//   odeCollision->SetSpaceId(this->raySpaceId);
// 
//   RTQL8RayShapePtr ray(new RTQL8RayShape(odeCollision));
//   odeCollision->SetShape(ray);
// 
//   ray->SetPoints(_start, _end);
//   this->rays.push_back(ray);
// }
