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
#include "physics/ode/ODEBody.hh"
#include "physics/ode/ODEGeom.hh"
#include "physics/ode/ODEPhysics.hh"
#include "physics/ode/ODERayShape.hh"
#include "physics/ode/ODEMultiRayShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
ODEMultiRayShape::ODEMultiRayShape(Geom *parent)
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

  ODEBody *pBody = (ODEBody*)(this->geomParent->GetBody());
  pBody->SetSpaceId( this->raySpaceId );
  ((ODEGeom*)parent)->SetSpaceId(this->raySpaceId);
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
  ODEPhysics *ode = dynamic_cast<ODEPhysics*>(
      this->GetWorld()->GetPhysicsEngine());

  if (ode == NULL)
    gzthrow( "Invalid physics engine. Must use ODE." );

  ode->LockMutex();

  // Do collision detection
  dSpaceCollide2( ( dGeomID ) ( this->superSpaceId ),
      ( dGeomID ) ( ode->GetSpaceId() ),
      this, &UpdateCallback );
      
  ode->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
/// Ray-intersection callback
void ODEMultiRayShape::UpdateCallback( void *data, dGeomID o1, dGeomID o2 )
{
  int n = 0;
  dContactGeom contact;
  ODEGeom *geom1, *geom2 = NULL;
  ODEGeom *rayGeom = NULL;
  ODEGeom *hitGeom = NULL;
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
    geom1 = NULL;
    geom2 = NULL;

    // Get pointers to the underlying geoms
    if (dGeomGetClass(o1) == dGeomTransformClass)
      geom1 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      geom1 = (ODEGeom*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      geom2 = (ODEGeom*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      geom2 = (ODEGeom*) dGeomGetData(o2);

    assert(geom1 && geom2);

    rayGeom = NULL;
    hitGeom = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayGeom.
    if (dGeomGetClass(o1) == dRayClass)
    {
      rayGeom = (ODEGeom*) geom1;
      hitGeom = (ODEGeom*) geom2;
      dGeomRaySetParams(o1, 0, 0);
      dGeomRaySetClosestHit(o1, 1);
    }

    if (dGeomGetClass(o2) == dRayClass)
    {
      assert(rayGeom == NULL);
      rayGeom = (ODEGeom*) geom2;
      hitGeom = (ODEGeom* )geom1;
      dGeomRaySetParams(o2, 0, 0);
      dGeomRaySetClosestHit(o2, 1);
    }

    // Check for ray/geom intersections
    if (rayGeom && hitGeom)
    {
      n = dCollide(o1, o2, 1, &contact, sizeof(contact));

      if ( n > 0 )
      {
        RayShape *shape = (RayShape*)(rayGeom->GetShape());
        if (contact.depth < shape->GetLength())
        {
          shape->SetLength(contact.depth );
          shape->SetRetro( hitGeom->GetLaserRetro() );
          shape->SetFiducial( hitGeom->GetLaserFiducialId() );
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the geom
void ODEMultiRayShape::AddRay(const common::Vector3 &start, const common::Vector3 &end )
{
  MultiRayShape::AddRay(start,end);
  ODEGeom *odeGeom = new ODEGeom(this->geomParent->GetBody());
  odeGeom->SetName("ODE Ray Geom");

  ODERayShape *ray = new ODERayShape(odeGeom, **this->displayTypeP == "lines" );

  ray->SetPoints(start,end);
  this->rays.push_back(ray);
}

