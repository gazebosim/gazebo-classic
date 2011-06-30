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
