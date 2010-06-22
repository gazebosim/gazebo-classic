#ifndef ODEBOXSHAPE_HH
#define ODEBOXSHAPE_HH

#include "ODEPhysics.hh"
#include "Mass.hh"
#include "Vector3.hh"
#include "BoxShape.hh"

namespace gazebo
{
  class ODEBoxShape : public BoxShape
  {
    public: ODEBoxShape(Geom *parent) : BoxShape(parent) {}
    public: virtual ~ODEBoxShape() {}
    public: virtual void SetSize( const Vector3 &size )
            {
              BoxShape::SetSize(size);
              PhysicsEngine *physics = World::Instance()->GetPhysicsEngine();
              ODEGeom *oParent = (ODEGeom*)(this->parent);

              Pose3d rpose;

              dMass odeMass;

              Mass mass = this->parent->GetMass();
  
              // Initialize box mass matrix
              dMassSetBoxTotal(&odeMass, mass.GetAsDouble(), 
                               size.x, size.y, size.z);
              rpose = this->parent->GetRelativePose();
              dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
 
              physics->ConvertMass(&mass, &odeMass);

              this->parent->SetMass(mass);
  
              // Create a box geometry with box mass matrix
              oParent->SetGeom( dCreateBox( 0, size.x, size.y, size.z), true );
            }
  
  
  };
}
#endif
