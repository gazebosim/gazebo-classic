/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
}

//////////////////////////////////////////////////
ODEMultiRayShape::ODEMultiRayShape(PhysicsEnginePtr _physicsEngine)
: MultiRayShape(_physicsEngine)
{
  this->defaultUpdate = false;

  this->SetName("ODE Multiray Shape");

  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate(0);

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  this->SetWorld(_physicsEngine->World());
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

  if (ode == nullptr)
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
  ODEMultiRayShape *self = nullptr;

  self = static_cast<ODEMultiRayShape*>(_data);

  // Check space
  if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
  {
    if (dGeomGetSpace(_o1) == self->superSpaceId ||
        dGeomGetSpace(_o2) == self->superSpaceId)
    {
      dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
    }

    if (dGeomGetSpace(_o1) == self->raySpaceId ||
        dGeomGetSpace(_o2) == self->raySpaceId)
    {
      dSpaceCollide2(_o1, _o2, self, &UpdateCallback);
    }
  }
  else
  {
    ODECollision *collision1 = nullptr;
    ODECollision *collision2 = nullptr;
    dGeomID rayId = 0;

    // Get pointers to the underlying collisions
    if (dGeomGetClass(_o1) == dGeomTransformClass)
    {
      collision1 = static_cast<ODECollision*>(
          dGeomGetData(dGeomTransformGetGeom(_o1)));
    }
    else
    {
      collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));
    }

    if (dGeomGetClass(_o2) == dGeomTransformClass)
    {
      collision2 =
        static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
    }
    else
    {
      collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));
    }

    ODECollision *rayCollision = nullptr;
    ODECollision *hitCollision = nullptr;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayCollision.
    if (dGeomGetClass(_o1) == dRayClass)
    {
      rayCollision = collision1;
      rayId = _o1;
      hitCollision = collision2;
      dGeomRaySetParams(_o1, 0, 0);
      dGeomRaySetClosestHit(_o1, 1);
    }
    else if (dGeomGetClass(_o2) == dRayClass)
    {
      GZ_ASSERT(rayCollision == nullptr, "rayCollision is not null");
      rayCollision = collision2;
      hitCollision = collision1;
      rayId = _o2;
      dGeomRaySetParams(_o2, 0, 0);
      dGeomRaySetClosestHit(_o2, 1);
    }

    if (!self->defaultUpdate || (rayCollision && hitCollision))
    {
      int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));

      if (n > 0 && self->defaultUpdate)
      {
        RayShape *shape = self->defaultUpdate ?
          boost::static_pointer_cast<RayShape>(rayCollision->GetShape()).get() :
          static_cast<RayShape*>(dGeomGetData(rayId));

        if (shape && hitCollision && contact.depth < shape->GetLength())
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
          shape->SetCollisionName(hitCollision->GetScopedName());
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

  ODECollisionPtr odeCollision;
  ODERayShapePtr ray;

  // The collisionParent will exist in instances where the multiray is
  // attached to an object, such as in the case of laser range finders
  if (this->collisionParent)
  {
    odeCollision.reset(new ODECollision(this->collisionParent->GetLink()));
    odeCollision->SetName("ode_ray_collision");
    odeCollision->SetSpaceId(this->raySpaceId);
    ray.reset(new ODERayShape(odeCollision));
    odeCollision->SetShape(ray);
  }
  // The else clause is run when a standalone multiray shape is
  // instantiated.
  // See test/integration/multirayshape.cc for an example.
  else
  {
    ray.reset(new ODERayShape(boost::dynamic_pointer_cast<ODEPhysics>(
            this->GetWorld()->GetPhysicsEngine()), this->raySpaceId));
    dGeomSetData(ray->ODEGeomId(), ray.get());
  }

  ray->SetPoints(_start, _end);
  this->rays.push_back(ray);
}
