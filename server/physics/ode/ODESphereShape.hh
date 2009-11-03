#ifndef ODESPHERESHAPE_HH
#define ODESPHERESHAPE_HH

#include <ode/ode.h>

#include "Mass.hh"
#include "PhysicsEngine.hh"
#include "SphereShape.hh"

namespace gazebo
{
  class ODESphereShape : public SphereShape
  {
    public: ODESphereShape(Geom *parent) : SphereShape(parent) {}
    public: virtual ~ODESphereShape() {}
    public: void SetSize(const double &radius)
            {
              SphereShape::SetSize(radius);
              PhysicsEngine *physics = World::Instance()->GetPhysicsEngine();
              ODEGeom *oParent = (ODEGeom*)(this->parent);

              dMass odeMass;
              Pose3d rpose;
  
              Mass mass = this->parent->GetMass();
  
              // Initialize box mass matrix
              dMassSetSphereTotal(&odeMass, mass.GetAsDouble(), radius);
              rpose = this->parent->GetRelativePose();
              dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
  
              physics->ConvertMass(&mass, &odeMass);
              
              this->parent->SetMass(mass);
  
              // Create the sphere geometry
              oParent->SetGeom( dCreateSphere(0, radius), true);
            }
  
  };
}
#endif
