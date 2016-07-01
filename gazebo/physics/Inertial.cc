/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include "gazebo/physics/Inertial.hh"

using namespace gazebo;
using namespace physics;

sdf::ElementPtr Inertial::sdfInertial;

//////////////////////////////////////////////////
Inertial::Inertial()
{
  this->mass = 1;
  this->principals.Set(1, 1, 1);
  this->products.Set(0, 0, 0);

  if (!this->sdfInertial)
  {
    this->sdfInertial.reset(new sdf::Element);
    initFile("inertial.sdf", this->sdfInertial);
  }

  // This is the only time this->sdfInertial should be used.
  this->sdf = this->sdfInertial->Clone();
}

//////////////////////////////////////////////////
Inertial::Inertial(double _m)
{
  this->sdf.reset(new sdf::Element);
  initFile("inertial.sdf", this->sdf);

  this->mass = _m;
  this->cog.Set(0, 0, 0, 0, 0, 0);
  this->principals.Set(1, 1, 1);
  this->products.Set(0, 0, 0);
}

//////////////////////////////////////////////////
Inertial::Inertial(const Inertial &_inertial)
{
  this->sdf.reset(new sdf::Element);
  initFile("inertial.sdf", this->sdf);

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
void Inertial::UpdateParameters(sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;

  // use default pose (identity) if not specified in sdf
  math::Pose pose = this->sdf->Get<math::Pose>("pose");
  this->SetCoG(pose);

  // if (this->sdf->HasElement("inertia"))
  // Do the following whether an inertia element was specified or not.
  // Otherwise SetUpdateFunc won't get called.
  {
    sdf::ElementPtr inertiaElem = this->sdf->GetElement("inertia");
    this->SetInertiaMatrix(
        inertiaElem->Get<double>("ixx"),
        inertiaElem->Get<double>("iyy"),
        inertiaElem->Get<double>("izz"),
        inertiaElem->Get<double>("ixy"),
        inertiaElem->Get<double>("ixz"),
        inertiaElem->Get<double>("iyz"));

    inertiaElem->GetElement("ixx")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIXX, this));
    inertiaElem->GetElement("iyy")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIYY, this));
    inertiaElem->GetElement("izz")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIZZ, this));
    inertiaElem->GetElement("ixy")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIXY, this));
    inertiaElem->GetElement("ixz")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIXZ, this));
    inertiaElem->GetElement("iyz")->GetValue()->SetUpdateFunc(
        boost::bind(&Inertial::GetIYZ, this));
  }

  this->SetMass(this->sdf->Get<double>("mass"));
  this->sdf->GetElement("mass")->GetValue()->SetUpdateFunc(
      boost::bind(&Inertial::GetMass, this));
}

//////////////////////////////////////////////////
Inertial Inertial::GetInertial(const math::Pose &_frameOffset) const
{
  // make a copy of the current Inertial
  Inertial result(*this);

  // new CoG location after link frame offset
  result.cog = result.cog - _frameOffset;

  // new MOI after link frame offset
  result.SetMOI(this->GetMOI(result.cog));

  return result;
}

//////////////////////////////////////////////////
void Inertial::Reset()
{
  sdf::ElementPtr inertiaElem = this->sdf->GetElement("inertia");

  this->mass = this->sdf->Get<double>("mass");
  this->cog.Set(0, 0, 0, 0, 0, 0);
  this->SetInertiaMatrix(
        inertiaElem->Get<double>("ixx"),
        inertiaElem->Get<double>("iyy"),
        inertiaElem->Get<double>("izz"),

        inertiaElem->Get<double>("ixy"),
        inertiaElem->Get<double>("ixz"),
        inertiaElem->Get<double>("iyz"));
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
  this->cog.pos.Set(_cx, _cy, _cz);
}

//////////////////////////////////////////////////
void Inertial::SetCoG(const math::Vector3 &_c)
{
  this->cog.pos = _c;
}

//////////////////////////////////////////////////
void Inertial::SetCoG(double _cx, double _cy, double _cz,
                      double _rx, double _ry, double _rz)
{
  this->cog.Set(_cx, _cy, _cz, _rx, _ry, _rz);
}

//////////////////////////////////////////////////
void Inertial::SetCoG(const math::Pose &_c)
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
void Inertial::SetMOI(const math::Matrix3 &_moi)
{
  /// \TODO: check symmetry of incoming _moi matrix
  this->principals.Set(_moi[0][0], _moi[1][1], _moi[2][2]);
  this->products.Set(_moi[0][1], _moi[0][2], _moi[1][2]);
}

//////////////////////////////////////////////////
math::Matrix3 Inertial::GetMOI() const
{
  return math::Matrix3(
    this->principals.x, this->products.x,   this->products.y,
    this->products.x,   this->principals.y, this->products.z,
    this->products.y,   this->products.z,   this->principals.z);
}

//////////////////////////////////////////////////
void Inertial::Rotate(const math::Quaternion &_rot)
{
  /// \TODO: double check what this does, if needed
  this->cog.pos = _rot.RotateVector(this->cog.pos);
  this->cog.rot = _rot * this->cog.rot;
}

//////////////////////////////////////////////////
Inertial &Inertial::operator=(const Inertial &_inertial)
{
  this->mass = _inertial.mass;
  this->cog = _inertial.cog;
  this->principals = _inertial.principals;
  this->products = _inertial.products;

  return *this;
}

//////////////////////////////////////////////////
Inertial Inertial::operator+(const Inertial &_inertial) const
{
  Inertial result(*this);

  // update mass with sum
  result.mass = this->mass + _inertial.mass;

  // compute new center of mass
  result.cog.pos =
    (this->cog.pos*this->mass + _inertial.cog.pos * _inertial.mass) /
    result.mass;

  // make a decision on the new orientation, set it to identity
  result.cog.rot = math::Quaternion(1, 0, 0, 0);

  // compute equivalent I for (*this) at the new CoG
  math::Matrix3 Ithis = this->GetMOI(result.cog);

  // compute equivalent I for _inertial at the new CoG
  math::Matrix3 Iparam = _inertial.GetMOI(result.cog);

  // sum up principals and products now they are at the same location
  result.SetMOI(Ithis + Iparam);

  return result;
}

//////////////////////////////////////////////////
math::Matrix3 Inertial::GetMOI(const math::Pose &_pose) const
{
  // get MOI as a Matrix3
  math::Matrix3 moi = this->GetMOI();

  // transform from new _pose to old this->cog, specified in new _pose frame
  math::Pose new2Old = this->cog - _pose;

  // rotate moi into new cog frame
  moi = new2Old.rot.GetAsMatrix3() * moi *
        new2Old.rot.GetInverse().GetAsMatrix3();

  // parallel axis theorem to get MOI at the new cog location
  // integrating point mass at some offset
  math::Vector3 offset = new2Old.pos;
  moi[0][0] += (offset.y * offset.y + offset.z * offset.z) * this->mass;
  moi[0][1] -= (offset.x * offset.y) * this->mass;
  moi[0][2] -= (offset.x * offset.z) * this->mass;
  moi[1][0] -= (offset.y * offset.x) * this->mass;
  moi[1][1] += (offset.x * offset.x + offset.z * offset.z) * this->mass;
  moi[1][2] -= (offset.y * offset.z) * this->mass;
  moi[2][0] -= (offset.z * offset.x) * this->mass;
  moi[2][1] -= (offset.z * offset.y) * this->mass;
  moi[2][2] += (offset.x * offset.x + offset.y * offset.y) * this->mass;

  return moi;
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
  if (_msg.has_ixx())
    this->SetIXX(_msg.ixx());
  if (_msg.has_ixy())
    this->SetIXY(_msg.ixy());
  if (_msg.has_ixz())
    this->SetIXZ(_msg.ixz());
  if (_msg.has_iyy())
    this->SetIYY(_msg.iyy());
  if (_msg.has_iyz())
    this->SetIYZ(_msg.iyz());
  if (_msg.has_izz())
    this->SetIZZ(_msg.izz());
}
