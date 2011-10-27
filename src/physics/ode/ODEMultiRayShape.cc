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
#include "physics/World.hh"
#include "physics/ode/ODETypes.hh"
#include "physics/ode/ODELink.hh"
#include "physics/ode/ODECollision.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODERayShape.hh"
#include "physics/ode/ODEMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
ODEMultiRayShape::ODEMultiRayShape(CollisionPtr parent)
  : MultiRayShape(parent)
{
  this->SetName("ODE Multiray Shape");

  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate( 0 );

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate( this->superSpaceId );

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  ODELinkPtr pLink = boost::shared_static_cast<ODELink>(this->collisionParent->GetLink());
  pLink->SetSpaceId( this->raySpaceId );
  boost::shared_static_cast<ODECollision>(parent)->SetSpaceId(this->raySpaceId);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
ODEMultiRayShape::~ODEMultiRayShape()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the rays 
void ODEMultiRayShape::UpdateRays()
{
  ODEPhysicsPtr ode = boost::shared_dynamic_cast<ODEPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (ode == NULL)
    gzthrow( "Invalid physics engine. Must use ODE." );

  // FIXME: Do we need to lock the physics engine here? YES!
  //        especially when spawning models with sensors
  
  // Do collision detection
  dSpaceCollide2( ( dGeomID ) ( this->superSpaceId ),
      ( dGeomID ) ( ode->GetSpaceId() ),
      this, &UpdateCallback );
}

////////////////////////////////////////////////////////////////////////////////
/// Ray-intersection callback
void ODEMultiRayShape::UpdateCallback( void *data, dGeomID o1, dGeomID o2 )
{
  dContactGeom contact;
  ODECollision *collision1, *collision2 = NULL;
  ODECollision *rayCollision = NULL;
  ODECollision *hitCollision = NULL;
  ODEMultiRayShape *self = NULL;

  self = (ODEMultiRayShape*) data;

  // Check space
  if ( dGeomIsSpace( o1 ) || dGeomIsSpace( o2 ) )
  {
    if (dGeomGetSpace(o1) == self->superSpaceId || 
        dGeomGetSpace(o2) == self->superSpaceId)
      dSpaceCollide2( o1, o2, self, &UpdateCallback );

    if (dGeomGetSpace(o1) == self->raySpaceId || 
        dGeomGetSpace(o2) == self->raySpaceId)
      dSpaceCollide2( o1, o2, self, &UpdateCallback );
  }
  else
  {
    collision1 = NULL;
    collision2 = NULL;

    // Get pointers to the underlying collisions
    if (dGeomGetClass(o1) == dGeomTransformClass)
      collision1 = (ODECollision*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      collision1 = (ODECollision*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      collision2 = (ODECollision*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      collision2 = (ODECollision*) dGeomGetData(o2);

    assert(collision1 && collision2);

    rayCollision = NULL;
    hitCollision = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayCollision.
    if (dGeomGetClass(o1) == dRayClass)
    {
      rayCollision = (ODECollision*) collision1;
      hitCollision = (ODECollision*) collision2;
      dGeomRaySetParams(o1, 0, 0);
      dGeomRaySetClosestHit(o1, 1);
    }
    else if (dGeomGetClass(o2) == dRayClass)
    {
      assert(rayCollision == NULL);
      rayCollision = (ODECollision*) collision2;
      hitCollision = (ODECollision*) collision1;
      dGeomRaySetParams(o2, 0, 0);
      dGeomRaySetClosestHit(o2, 1);
    }

    // Check for ray/collision intersections
    if (rayCollision && hitCollision)
    {
      int n = dCollide(o1, o2, 1, &contact, sizeof(contact));

      if ( n > 0 )
      {
        RayShapePtr shape = boost::shared_static_cast<RayShape>(rayCollision->GetShape());
        if (contact.depth < shape->GetLength())
        {
          //gzerr << "ODEMultiRayShape UpdateCallback dSpaceCollide2 "
          //      << " depth[" << contact.depth << "]"
          //      << " position[" << contact.pos[0]
          //        << "," << contact.pos[1]
          //        << "," << contact.pos[2]
          //        << "," << "]"
          //      << " ray[" << rayCollision->GetName() << "]"
          //      << " pose[" << rayCollision->GetWorldPose() << "]"
          //      << " hit[" << hitCollision->GetName() << "]"
          //      << " pose[" << hitCollision->GetWorldPose() << "]"
          //      << "\n";
          shape->SetLength(contact.depth );
          shape->SetRetro( hitCollision->GetLaserRetro() );
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the collision
void ODEMultiRayShape::AddRay(const math::Vector3 &start, const math::Vector3 &end )
{
  MultiRayShape::AddRay(start,end);
  ODECollisionPtr odeCollision( new ODECollision(this->collisionParent->GetLink()) );
  odeCollision->SetName("ODE Ray Collision");
  odeCollision->SetSpaceId(this->raySpaceId);

  ODERayShapePtr ray( new ODERayShape(odeCollision, true ));
  odeCollision->SetShape(ray);

  ray->SetPoints(start,end);
  this->rays.push_back(ray);
}

