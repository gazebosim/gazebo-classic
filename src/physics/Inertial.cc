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
#include "sdf/sdf_parser.h"
#include "Inertial.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Inertial::Inertial()
{
  this->mass = 0;
  this->principals.Set(0, 0, 0);
  this->products.Set(0, 0, 0);

  this->sdf.reset(new sdf::Element);
  sdf::initFile("/sdf/inertial.sdf", this->sdf);
}

//////////////////////////////////////////////////
Inertial::Inertial(double _m)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("/sdf/inertial.sdf", this->sdf);

  this->mass = _m;
  this->cog.Set(0, 0, 0);
  this->principals.Set(0, 0, 0);
  this->products.Set(0, 0, 0);
}

//////////////////////////////////////////////////
Inertial::Inertial(const Inertial &_inertial)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("/sdf/inertial.sdf", this->sdf);

  (*this) = _inertial;
}

//////////////////////////////////////////////////
Inertial::~Inertial()
{
  this->sdf.reset();
}

//////////////////////////////////////////////////
void Inertial::Load(sdf::ElementPtr _sdf)
{
  this->UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
void Inertial::UpdateParameters(sdf::ElementPtr &_sdf)
{
  this->sdf = _sdf;

  math::Vector3 center(0, 0, 0);
  if (this->sdf->HasElement("origin"))
  {
    center = this->sdf->GetElement("origin")->GetValuePose("pose").pos;
    this->sdf->GetElement("origin")->GetAttribute("pose")->SetUpdateFunc(
        boost::bind(&Inertial::GetPose, this));
  }
  this->SetCoG(center.x, center.y, center.z);

  sdf::ElementPtr inertiaElem = this->sdf->GetElement("inertia");
  this->SetInertiaMatrix(
      inertiaElem->GetValueDouble("ixx"),
      inertiaElem->GetValueDouble("iyy"),
      inertiaElem->GetValueDouble("izz"),
      inertiaElem->GetValueDouble("ixy"),
      inertiaElem->GetValueDouble("ixz"),
      inertiaElem->GetValueDouble("iyz"));

  inertiaElem->GetAttribute("ixx")->SetUpdateFunc(
      boost::bind(&Inertial::GetIXX, this));
  inertiaElem->GetAttribute("iyy")->SetUpdateFunc(
      boost::bind(&Inertial::GetIYY, this));
  inertiaElem->GetAttribute("izz")->SetUpdateFunc(
      boost::bind(&Inertial::GetIZZ, this));
  inertiaElem->GetAttribute("ixy")->SetUpdateFunc(
      boost::bind(&Inertial::GetIXY, this));
  inertiaElem->GetAttribute("ixz")->SetUpdateFunc(
      boost::bind(&Inertial::GetIXZ, this));
  inertiaElem->GetAttribute("iyz")->SetUpdateFunc(
      boost::bind(&Inertial::GetIYZ, this));

  this->SetMass(this->sdf->GetValueDouble("mass"));
  this->sdf->GetAttribute("mass")->SetUpdateFunc(
      boost::bind(&Inertial::GetMass, this));
}

//////////////////////////////////////////////////
void Inertial::SetLinearDamping(double _damping)
{
  this->sdf->GetOrCreateElement("damping")->GetAttribute("linear")->Set(
      _damping);
}

//////////////////////////////////////////////////
void Inertial::SetAngularDamping(double _damping)
{
  this->sdf->GetOrCreateElement("damping")->GetAttribute("angular")->Set(
      _damping);
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
void Inertial::Reset()
{
  this->mass = 0;
  this->cog.Set(0, 0, 0);
  this->principals.Set(0, 0, 0);
  this->products.Set(0, 0, 0);
}

//////////////////////////////////////////////////
void Inertial::SetMass(double _m)
{
  this->mass = _m;
}

//////////////////////////////////////////////////
double Inertial::GetMass() const
{
  return this->mass;
}

//////////////////////////////////////////////////
void Inertial::SetCoG(double _cx, double _cy, double _cz)
{
  this->cog.Set(_cx, _cy, _cz);
}

//////////////////////////////////////////////////
void Inertial::SetCoG(const math::Vector3 &_c)
{
  this->cog = _c;
}

//////////////////////////////////////////////////
void Inertial::SetInertiaMatrix(double ixx, double iyy, double izz,
                                double ixy, double ixz, double iyz)
{
  this->principals.Set(ixx, iyy, izz);
  this->products.Set(ixy, ixz, iyz);
}


//////////////////////////////////////////////////
math::Vector3 Inertial::GetPrincipalMoments() const
{
  return this->principals;
}

//////////////////////////////////////////////////
math::Vector3 Inertial::GetProductsofInertia() const
{
  return this->products;
}

//////////////////////////////////////////////////
void Inertial::Rotate(const math::Quaternion &_rot)
{
  this->cog = _rot.RotateVector(this->cog);
}

//////////////////////////////////////////////////
void Inertial::operator =(const Inertial &_inertial)
{
  this->mass = _inertial.mass;
  this->cog = _inertial.cog;
  this->principals = _inertial.principals;
  this->products = _inertial.products;
}

//////////////////////////////////////////////////
Inertial Inertial::operator+(const Inertial &_inertial) const
{
  Inertial result;
  result.mass = this->mass + _inertial.mass;

  result.cog = (this->cog*this->mass + _inertial.cog * _inertial.mass) /
                result.mass;

  result.principals = this->principals + _inertial.principals;
  result.products = this->products + _inertial.products;
  return result;
}

//////////////////////////////////////////////////
const Inertial &Inertial::operator+=(const Inertial &_inertial)
{
  *this = *this + _inertial;
  return *this;
}

//////////////////////////////////////////////////
double Inertial::GetIXX() const
{
  return this->principals.x;
}

//////////////////////////////////////////////////
double Inertial::GetIYY() const
{
  return this->principals.y;
}

//////////////////////////////////////////////////
double Inertial::GetIZZ() const
{
  return this->principals.z;
}

//////////////////////////////////////////////////
double Inertial::GetIXY() const
{
  return this->products.x;
}

//////////////////////////////////////////////////
double Inertial::GetIXZ() const
{
  return this->products.y;
}

//////////////////////////////////////////////////
double Inertial::GetIYZ() const
{
  return this->products.z;
}

//////////////////////////////////////////////////
void Inertial::SetIXX(double _v)
{
  this->principals.x = _v;
}

//////////////////////////////////////////////////
void Inertial::SetIYY(double _v)
{
  this->principals.y = _v;
}

//////////////////////////////////////////////////
void Inertial::SetIZZ(double _v)
{
  this->principals.z = _v;
}

//////////////////////////////////////////////////
void Inertial::SetIXY(double _v)
{
  this->products.x = _v;
}

//////////////////////////////////////////////////
void Inertial::SetIXZ(double _v)
{
  this->products.y = _v;
}

//////////////////////////////////////////////////
void Inertial::SetIYZ(double _v)
{
  this->products.z = _v;
}

//////////////////////////////////////////////////
void Inertial::ProcessMsg(const msgs::Inertial &_msg)
{
  if (_msg.has_mass())
    this->SetMass(_msg.mass());
  if (_msg.has_pose())
    this->SetCoG(_msg.pose().position().x(),
                 _msg.pose().position().y(),
                 _msg.pose().position().z());
  if (_msg.has_linear_damping())
    this->SetLinearDamping(_msg.linear_damping());
  if (_msg.has_angular_damping())
    this->SetAngularDamping(_msg.angular_damping());
  if(_msg.has_ixx())
    this->SetIXX(_msg.ixx());
  if(_msg.has_ixy())
    this->SetIXY(_msg.ixy());
  if(_msg.has_ixz())
    this->SetIXZ(_msg.ixz());
  if(_msg.has_iyy())
    this->SetIYY(_msg.iyy());
  if(_msg.has_iyz())
    this->SetIYZ(_msg.iyz());
  if(_msg.has_izz())
    this->SetIZZ(_msg.izz());
}

