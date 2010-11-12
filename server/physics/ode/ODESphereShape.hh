#ifndef ODESPHERESHAPE_HH
#define ODESPHERESHAPE_HH

#include "ODEPhysics.hh"
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
              PhysicsEngine *physics = this->GetWorld()->GetPhysicsEngine();
              ODEGeom *oParent = (ODEGeom*)(this->geomParent);

              dMass odeMass;
              Pose3d rpose;
  
              Mass mass = this->geomParent->GetMass();
  
              // Initialize box mass matrix
              dMassSetSphereTotal(&odeMass, mass.GetAsDouble(), radius);
              rpose = this->geomParent->GetRelativePose();
              dMassTranslate(&odeMass, rpose.pos.x, rpose.pos.y, rpose.pos.z);
  
              physics->ConvertMass(&mass, &odeMass);
              
              this->geomParent->SetMass(mass);
  
              // Create the sphere geometry
              oParent->SetGeom( dCreateSphere(0, radius), true);
            }
  
  };
}
#endif
