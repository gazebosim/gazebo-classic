#include "Inertial.hh"

using namespace gazebo;
using namespace physics;

void Inertial::Load( sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;

  math::Vector3 center(0,0,0);
  if (this->sdf->HasElement("origin"))
    center = this->sdf->GetElement("origin")->GetValuePose("pose").pos;
  this->mass.SetCoG(center.x, center.y, center.z);

 
  sdf::ElementPtr inertiaElem = this->sdf->GetElement("inertia"); 
  this->mass.SetInertiaMatrix( 
      inertiaElem->GetValueDouble("ixx"),
      inertiaElem->GetValueDouble("iyy"),
      inertiaElem->GetValueDouble("izz"),
      inertiaElem->GetValueDouble("ixy"),
      inertiaElem->GetValueDouble("ixz"),
      inertiaElem->GetValueDouble("iyz"));
      
  this->mass.SetMass( this->sdf->GetValueDouble("mass") );
}

double Inertial::GetLinearDamping()
{
  double value = 0;
  if (this->sdf->HasElement("damping"))
  {
    sdf::ElementPtr dampingElem = this->sdf->GetElement("damping");
    value = dampingElem->GetValueDouble("linear");
  }

  return value;
}

double Inertial::GetAngularDamping()
{
  double value = 0;
  if (this->sdf->HasElement("damping"))
  {
    sdf::ElementPtr dampingElem = this->sdf->GetElement("damping");
    value = dampingElem->GetValueDouble("angular");
  }

  return value;
}

const Mass &Inertial::GetMass() const
{
  return this->mass;
}
