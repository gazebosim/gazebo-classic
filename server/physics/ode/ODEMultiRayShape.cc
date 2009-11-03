#include "ODEBody.hh"
#include "ODEGeom.hh"
#include "World.hh"
#include "ODEPhysics.hh"
#include "ODERayShape.hh"
#include "ODEMultiRayShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
ODEMultiRayShape::ODEMultiRayShape(Geom *parent)
  : MultiRayShape(parent)
{
  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate( 0 );

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate( this->superSpaceId );

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  ODEBody *pBody = (ODEBody*)(this->parent->GetBody());
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
      World::Instance()->GetPhysicsEngine());

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
void ODEMultiRayShape::AddRay(const Vector3 &start, const Vector3 &end )
{
  MultiRayShape::AddRay(start,end);

  ODERayShape *ray = new ODERayShape( new ODEGeom(parent->GetBody()), **this->displayTypeP == "lines");

  ray->SetPoints(start,end);
  this->rays.push_back(ray);
}

