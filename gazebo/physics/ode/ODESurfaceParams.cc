/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODESurfaceParams::ODESurfaceParams()
  : SurfaceParams(),
    bounce(0), bounceThreshold(100000),
    kp(1000000000000), kd(1), cfm(0), erp(0.2),
    maxVel(0.01), minDepth(0),
    slip1(0), slip2(0),
    frictionPyramid(new physics::FrictionPyramid())
{
}

//////////////////////////////////////////////////
ODESurfaceParams::~ODESurfaceParams()
{
}

//////////////////////////////////////////////////
void ODESurfaceParams::Load(sdf::ElementPtr _sdf)
{
  // Load parent class
  SurfaceParams::Load(_sdf);

  if (!_sdf)
    gzerr << "Surface _sdf is NULL" << std::endl;
  else
  {
    {
      sdf::ElementPtr bounceElem = _sdf->GetElement("bounce");
      if (!bounceElem)
        gzerr << "Surface bounce sdf member is NULL" << std::endl;
      else
      {
        this->bounce = bounceElem->Get<double>("restitution_coefficient");
        if (this->bounce < 0)
        {
          gzwarn << "bounce restitution_coefficient ["
                 << this->bounce
                 << "] < 0, so it will not be applied by ODE."
                 << std::endl;
        }
        else if (this->bounce > 1)
        {
          gzwarn << "bounce restitution_coefficient ["
                 << this->bounce
                 << "] > 1, which is outside the recommended range."
                 << std::endl;
        }
        this->bounceThreshold = bounceElem->Get<double>("threshold");
      }
    }

    {
      sdf::ElementPtr frictionElem = _sdf->GetElement("friction");
      if (!frictionElem)
        gzerr << "Surface friction sdf member is NULL" << std::endl;
      else
      {
        sdf::ElementPtr torsionalElem = frictionElem->GetElement("torsional");
        if (torsionalElem)
        {
          this->frictionPyramid->SetMuTorsion(
            torsionalElem->Get<double>("coefficient"));
          this->frictionPyramid->SetPatchRadius(
            torsionalElem->Get<double>("patch_radius"));
          this->frictionPyramid->SetSurfaceRadius(
            torsionalElem->Get<double>("surface_radius"));
          this->frictionPyramid->SetUsePatchRadius(
            torsionalElem->Get<bool>("use_patch_radius"));

          sdf::ElementPtr torsionalOdeElem = torsionalElem->GetElement("ode");
          if (torsionalOdeElem)
            this->slipTorsion = torsionalOdeElem->Get<double>("slip");
        }

        sdf::ElementPtr frictionOdeElem = frictionElem->GetElement("ode");
        if (!frictionOdeElem)
          gzerr << "Surface friction ode sdf member is NULL" << std::endl;
        else
        {
          this->frictionPyramid->SetMuPrimary(
            frictionOdeElem->Get<double>("mu"));
          this->frictionPyramid->SetMuSecondary(
            frictionOdeElem->Get<double>("mu2"));
          this->frictionPyramid->direction1 =
            frictionOdeElem->Get<math::Vector3>("fdir1");

          this->slip1 = frictionOdeElem->Get<double>("slip1");
          this->slip2 = frictionOdeElem->Get<double>("slip2");
        }
      }
    }

    {
      sdf::ElementPtr contactElem = _sdf->GetElement("contact");
      if (!contactElem)
        gzerr << "Surface contact sdf member is NULL" << std::endl;
      else
      {
        sdf::ElementPtr contactOdeElem = contactElem->GetElement("ode");
        if (!contactOdeElem)
          gzerr << "Surface contact ode sdf member is NULL" << std::endl;
        {
          this->kp = contactOdeElem->Get<double>("kp");
          this->kd = contactOdeElem->Get<double>("kd");
          this->cfm = contactOdeElem->Get<double>("soft_cfm");
          this->erp = contactOdeElem->Get<double>("soft_erp");
          this->maxVel = contactOdeElem->Get<double>("max_vel");
          this->minDepth = contactOdeElem->Get<double>("min_depth");
        }
        this->frictionPyramid->SetPoissonsRatio(
          contactElem->Get<double>("poissons_ratio"));
        this->frictionPyramid->SetElasticModulus(
          contactElem->Get<double>("elastic_modulus"));
      }
    }
  }
}

/////////////////////////////////////////////////
void ODESurfaceParams::FillMsg(msgs::Surface &_msg)
{
  SurfaceParams::FillMsg(_msg);

  _msg.mutable_friction()->set_mu(this->frictionPyramid->MuPrimary());
  _msg.mutable_friction()->set_mu2(this->frictionPyramid->MuSecondary());
  _msg.mutable_friction()->set_slip1(this->slip1);
  _msg.mutable_friction()->set_slip2(this->slip2);
  msgs::Set(_msg.mutable_friction()->mutable_fdir1(),
            this->frictionPyramid->direction1.Ign());

  _msg.mutable_friction()->mutable_torsional()->set_coefficient(
      this->frictionPyramid->MuTorsion());

  _msg.mutable_friction()->mutable_torsional()->set_patch_radius(
      this->frictionPyramid->PatchRadius());
  _msg.mutable_friction()->mutable_torsional()->set_surface_radius(
      this->frictionPyramid->SurfaceRadius());
  _msg.mutable_friction()->mutable_torsional()->set_use_patch_radius(
      this->frictionPyramid->UsePatchRadius());
  _msg.mutable_friction()->mutable_torsional()->mutable_ode()->set_slip(
      this->slipTorsion);

  _msg.set_restitution_coefficient(this->bounce);
  _msg.set_bounce_threshold(this->bounceThreshold);

  _msg.set_soft_cfm(this->cfm);
  _msg.set_soft_erp(this->erp);
  _msg.set_elastic_modulus(this->frictionPyramid->ElasticModulus());
  _msg.set_kp(this->kp);
  _msg.set_kd(this->kd);
  _msg.set_max_vel(this->maxVel);
  _msg.set_min_depth(this->minDepth);
}

/////////////////////////////////////////////////
void ODESurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  SurfaceParams::ProcessMsg(_msg);

  if (_msg.has_friction())
  {
    if (_msg.friction().has_mu())
      this->frictionPyramid->SetMuPrimary(_msg.friction().mu());
    if (_msg.friction().has_mu2())
      this->frictionPyramid->SetMuSecondary(_msg.friction().mu2());
    if (_msg.friction().has_slip1())
      this->slip1 = _msg.friction().slip1();
    if (_msg.friction().has_slip2())
      this->slip2 = _msg.friction().slip2();
    if (_msg.friction().has_fdir1())
    {
      this->frictionPyramid->direction1 =
        msgs::ConvertIgn(_msg.friction().fdir1());
    }

    if (_msg.friction().has_torsional())
    {
      if (_msg.friction().torsional().has_coefficient())
      {
        this->frictionPyramid->SetMuTorsion(
            _msg.friction().torsional().coefficient());
      }
      if (_msg.friction().torsional().has_patch_radius())
      {
        this->frictionPyramid->SetPatchRadius(
            _msg.friction().torsional().patch_radius());
      }
      if (_msg.friction().torsional().has_surface_radius())
      {
        this->frictionPyramid->SetSurfaceRadius(
            _msg.friction().torsional().surface_radius());
      }
      if (_msg.friction().torsional().has_use_patch_radius())
      {
        this->frictionPyramid->SetUsePatchRadius(
            _msg.friction().torsional().use_patch_radius());
      }
      if (_msg.friction().torsional().has_ode())
      {
        if (_msg.friction().torsional().ode().has_slip())
          this->slipTorsion = _msg.friction().torsional().ode().slip();
      }
    }
  }

  if (_msg.has_restitution_coefficient())
    this->bounce = _msg.restitution_coefficient();
  if (_msg.has_bounce_threshold())
    this->bounceThreshold = _msg.bounce_threshold();
  if (_msg.has_soft_cfm())
    this->cfm = _msg.soft_cfm();
  if (_msg.has_soft_erp())
    this->erp = _msg.soft_erp();
  if (_msg.has_elastic_modulus())
    this->frictionPyramid->SetElasticModulus(_msg.elastic_modulus());
  if (_msg.has_kp())
    this->kp = _msg.kp();
  if (_msg.has_kd())
    this->kd = _msg.kd();
  if (_msg.has_max_vel())
    this->maxVel = _msg.max_vel();
  if (_msg.has_min_depth())
    this->minDepth = _msg.min_depth();
}

/////////////////////////////////////////////////
FrictionPyramidPtr ODESurfaceParams::GetFrictionPyramid() const
{
  return this->frictionPyramid;
}

/////////////////////////////////////////////////
FrictionPyramidPtr ODESurfaceParams::FrictionPyramid() const
{
  return this->frictionPyramid;
}
