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
#include "gazebo/physics/plugin/PluginTypes.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginRayShape.hh"
#include "gazebo/physics/plugin/PluginMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PluginMultiRayShape::PluginMultiRayShape(CollisionPtr _parent)
  : MultiRayShape(_parent)
{
  this->SetName("Plugin Multiray Shape");

  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate(0);

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate(this->superSpaceId);

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  // These three lines may be unessecary
  PluginLinkPtr pLink =
    boost::static_pointer_cast<PluginLink>(this->collisionParent->GetLink());
  pLink->SetSpaceId(this->raySpaceId);
  boost::static_pointer_cast<PluginCollision>(this->collisionParent)->SetSpaceId(
      this->raySpaceId);
}

//////////////////////////////////////////////////
PluginMultiRayShape::~PluginMultiRayShape()
{
  dSpaceSetCleanup(this->raySpaceId, 0);
  dSpaceDestroy(this->raySpaceId);

  dSpaceSetCleanup(this->superSpaceId, 0);
  dSpaceDestroy(this->superSpaceId);
}

//////////////////////////////////////////////////
void PluginMultiRayShape::UpdateRays()
{
  PluginPhysicsPtr plugin = boost::dynamic_pointer_cast<PluginPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (plugin == NULL)
    gzthrow("Invalid physics engine. Must use Plugin.");

  // Do we need to lock the physics engine here? YES!
  // especially when spawning models with sensors
  {
    boost::recursive_mutex::scoped_lock lock(*plugin->GetPhysicsUpdateMutex());

    // Do collision detection
    dSpaceCollide2((dGeomID) (this->superSpaceId),
        (dGeomID) (plugin->GetSpaceId()),
        this, &UpdateCallback);
  }
}

//////////////////////////////////////////////////
void PluginMultiRayShape::UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2)
{
  dContactGeom contact;
  PluginMultiRayShape *self = NULL;

  self = static_cast<PluginMultiRayShape*>(_data);

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
    PluginCollision *collision1 = NULL;
    PluginCollision *collision2 = NULL;

    // Get pointers to the underlying collisions
    if (dGeomGetClass(_o1) == dGeomTransformClass)
    {
      collision1 = static_cast<PluginCollision*>(
          dGeomGetData(dGeomTransformGetGeom(_o1)));
    }
    else
      collision1 = static_cast<PluginCollision*>(dGeomGetData(_o1));

    if (dGeomGetClass(_o2) == dGeomTransformClass)
    {
      collision2 =
        static_cast<PluginCollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
    }
    else
    {
      collision2 = static_cast<PluginCollision*>(dGeomGetData(_o2));
    }

    GZ_ASSERT(collision1, "collision1 is null");
    GZ_ASSERT(collision2, "collision2 is null");

    PluginCollision *rayCollision = NULL;
    PluginCollision *hitCollision = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the Plugin dRayClass is used *soley* by the RayCollision.
    if (dGeomGetClass(_o1) == dRayClass)
    {
      rayCollision = static_cast<PluginCollision*>(collision1);
      hitCollision = static_cast<PluginCollision*>(collision2);
      dGeomRaySetParams(_o1, 0, 0);
      dGeomRaySetClosestHit(_o1, 1);
    }
    else if (dGeomGetClass(_o2) == dRayClass)
    {
      GZ_ASSERT(rayCollision == NULL, "rayCollision is not null");
      rayCollision = static_cast<PluginCollision*>(collision2);
      hitCollision = static_cast<PluginCollision*>(collision1);
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
          // gzerr << "PluginMultiRayShape UpdateCallback dSpaceCollide2 "
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
void PluginMultiRayShape::AddRay(const math::Vector3 &_start,
    const math::Vector3 &_end)
{
  MultiRayShape::AddRay(_start, _end);

  PluginCollisionPtr odeCollision(new PluginCollision(
        this->collisionParent->GetLink()));
  odeCollision->SetName("ode_ray_collision");
  odeCollision->SetSpaceId(this->raySpaceId);

  PluginRayShapePtr ray(new PluginRayShape(odeCollision));
  odeCollision->SetShape(ray);

  ray->SetPoints(_start, _end);
  this->rays.push_back(ray);
}
