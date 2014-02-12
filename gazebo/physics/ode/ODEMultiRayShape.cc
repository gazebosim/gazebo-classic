/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODERayShape.hh"
#include "gazebo/physics/ode/ODEMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEMultiRayShape::ODEMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent)
{
  this->SetName("ODE Multiray Shape");

  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate(0);

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  // These three lines may be unessecary
  ODELinkPtr pLink =
    boost::static_pointer_cast<ODELink>(this->collisionParent->GetLink());
  pLink->SetSpaceId(this->raySpaceId);
  boost::static_pointer_cast<ODECollision>(this->collisionParent)->SetSpaceId(
      this->raySpaceId);
}

//////////////////////////////////////////////////
ODEMultiRayShape::~ODEMultiRayShape()
{
  dSpaceSetCleanup(this->raySpaceId, 0);
  dSpaceDestroy(this->raySpaceId);

  dSpaceSetCleanup(this->superSpaceId, 0);
  dSpaceDestroy(this->superSpaceId);
}

//////////////////////////////////////////////////
void ODEMultiRayShape::UpdateRays()
{
  ODEPhysicsPtr ode = boost::dynamic_pointer_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (ode == NULL)
    gzthrow("Invalid physics engine. Must use ODE.");

  // Do we need to lock the physics engine here? YES!
  // especially when spawning models with sensors
  {
    boost::recursive_mutex::scoped_lock lock(*ode->GetPhysicsUpdateMutex());

    // Do collision detection
    dSpaceCollide2((dGeomID) (this->superSpaceId),
        (dGeomID) (ode->GetSpaceId()),
        this, &UpdateCallback);
  }
}

//////////////////////////////////////////////////
void ODEMultiRayShape::UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2)
{
  dContactGeom contact;
  ODEMultiRayShape *self = NULL;

  self = static_cast<ODEMultiRayShape*>(_data);

  // Check space
  if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
  {
    if (dGeomGetSpace(_o1) == self->superSpaceId ||
        dGeomGetSpace(_o2) == self->superSpaceId)
      dSpaceCollide2(_o1, _o2, self, &UpdateCallback);

    if (dGeomGetSpace(_o1) == self->raySpaceId ||
        dGeomGetSpace(_o2) == self->raySpaceId)
      dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
  }
  else
  {
    ODECollision *collision1 = NULL;
    ODECollision *collision2 = NULL;

    // Get pointers to the underlying collisions
    if (dGeomGetClass(_o1) == dGeomTransformClass)
    {
      collision1 = static_cast<ODECollision*>(
          dGeomGetData(dGeomTransformGetGeom(_o1)));
    }
    else
      collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));

    if (dGeomGetClass(_o2) == dGeomTransformClass)
    {
      collision2 =
        static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
    }
    else
    {
      collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));
    }

    GZ_ASSERT(collision1, "collision1 is null");
    GZ_ASSERT(collision2, "collision2 is null");

    ODECollision *rayCollision = NULL;
    ODECollision *hitCollision = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayCollision.
    if (dGeomGetClass(_o1) == dRayClass)
    {
      rayCollision = static_cast<ODECollision*>(collision1);
      hitCollision = static_cast<ODECollision*>(collision2);
      dGeomRaySetParams(_o1, 0, 0);
      dGeomRaySetClosestHit(_o1, 1);
    }
    else if (dGeomGetClass(_o2) == dRayClass)
    {
      GZ_ASSERT(rayCollision == NULL, "rayCollision is not null");
      rayCollision = static_cast<ODECollision*>(collision2);
      hitCollision = static_cast<ODECollision*>(collision1);
      dGeomRaySetParams(_o2, 0, 0);
      dGeomRaySetClosestHit(_o2, 1);
    }

    // Check for ray/collision intersections
    if (rayCollision && hitCollision)
    {
      int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));

      if (n > 0)
      {
        RayShapePtr shape = boost::static_pointer_cast<RayShape>(
            rayCollision->GetShape());
        if (contact.depth < shape->GetLength())
        {
          // gzerr << "ODEMultiRayShape UpdateCallback dSpaceCollide2 "
          //      << " depth[" << contact.depth << "]"
          //      << " position[" << contact.pos[0]
          //        << ", " << contact.pos[1]
          //        << ", " << contact.pos[2]
          //        << ", " << "]"
          //      << " ray[" << rayCollision->GetScopedName() << "]"
          //      << " pose[" << rayCollision->GetWorldPose() << "]"
          //      << " hit[" << hitCollision->GetScopedName() << "]"
          //      << " pose[" << hitCollision->GetWorldPose() << "]"
          //      << "\n";
          shape->SetLength(contact.depth);
          shape->SetRetro(hitCollision->GetLaserRetro());
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void ODEMultiRayShape::AddRay(const math::Vector3 &_start,
    const math::Vector3 &_end)
{
  MultiRayShape::AddRay(_start, _end);

  ODECollisionPtr odeCollision(new ODECollision(
        this->collisionParent->GetLink()));
  odeCollision->SetName("ode_ray_collision");
  odeCollision->SetSpaceId(this->raySpaceId);

  ODERayShapePtr ray(new ODERayShape(odeCollision));
  odeCollision->SetShape(ray);

  ray->SetPoints(_start, _end);
  this->rays.push_back(ray);
}
