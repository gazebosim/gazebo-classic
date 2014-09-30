/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginJoint.hh"
#include "gazebo/physics/GearboxJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/JointWrench.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
PluginJoint::PluginJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->jointId = NULL;
  this->implicitDampingState[0] = PluginJoint::NONE;
  this->implicitDampingState[1] = PluginJoint::NONE;
  this->stiffnessDampingInitialized = false;
  this->currentKd[0] = 0;
  this->currentKd[1] = 0;
  this->currentKp[0] = 0;
  this->currentKp[1] = 0;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
  this->useImplicitSpringDamper = false;
  this->stopERP = 0.0;
  this->stopCFM = 0.0;
}

//////////////////////////////////////////////////
PluginJoint::~PluginJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);

  this->Detach();
}

//////////////////////////////////////////////////
void PluginJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("physics") &&
      this->sdf->GetElement("physics")->HasElement("plugin"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("physics")->GetElement("plugin");

    if (elem->HasElement("implicit_spring_damper"))
    {
      this->useImplicitSpringDamper = elem->Get<bool>("implicit_spring_damper");
    }

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopERP = elem->GetElement("limit")->Get<double>("erp");
    for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
      this->SetParam("stop_erp", i, this->stopERP);

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopCFM = elem->GetElement("limit")->Get<double>("cfm");
    for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
      this->SetParam("stop_cfm", i, this->stopCFM);

    if (elem->HasElement("suspension"))
    {
    }

    if (elem->HasElement("fudge_factor"))
    {
    }

    if (elem->HasElement("cfm"))
        this->SetParam("cfm", 0, elem->Get<double>("cfm"));

    if (elem->HasElement("erp"))
        this->SetParam("erp", 0, elem->Get<double>("erp"));

    if (elem->HasElement("bounce"))
    {
    }

    if (elem->HasElement("max_force"))
    {
    }

    if (elem->HasElement("velocity"))
    {
    }
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }
}

//////////////////////////////////////////////////
LinkPtr PluginJoint::GetJointLink(unsigned int _index) const
{
  LinkPtr result;
  if (!this->jointId)
  {
    gzerr << "Plugin Joint ID is invalid\n";
    return result;
  }

  if (_index == 0 || _index == 1)
  {
    PluginLinkPtr odeLink1 = boost::static_pointer_cast<PluginLink>(this->childLink);
    PluginLinkPtr odeLink2 = boost::static_pointer_cast<PluginLink>(this->parentLink);
  }

  return result;
}

//////////////////////////////////////////////////
bool PluginJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  PluginLinkPtr odeLink1 = boost::dynamic_pointer_cast<PluginLink>(_one);
  PluginLinkPtr odeLink2 = boost::dynamic_pointer_cast<PluginLink>(_two);

  if (odeLink1 == NULL || odeLink2 == NULL)
    gzthrow("PluginJoint requires Plugin bodies\n");

  return false;
}

//////////////////////////////////////////////////
// child classes where appropriate
double PluginJoint::GetParam(unsigned int /*parameter*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void PluginJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  PluginLinkPtr odechild = boost::dynamic_pointer_cast<PluginLink>(this->childLink);
  PluginLinkPtr odeparent = boost::dynamic_pointer_cast<PluginLink>(this->parentLink);

  if (odechild == NULL && odeparent == NULL)
    gzthrow("PluginJoint requires at least one Plugin link\n");

  if (!this->jointId)
    gzerr << "Plugin Joint ID is invalid\n";

  if (this->HasType(Base::HINGE2_JOINT) &&
      (odechild == NULL || odeparent == NULL))
    gzthrow("PluginHinge2Joint cannot be connected to the world");

  if (!odechild && odeparent)
  {
  }
  else if (odechild && !odeparent)
  {
  }
  else if (odechild && odeparent)
  {
  }
}

//////////////////////////////////////////////////
void PluginJoint::Detach()
{
  Joint::Detach();
  this->childLink.reset();
  this->parentLink.reset();
}

//////////////////////////////////////////////////
// where appropriate
void PluginJoint::SetParam(unsigned int /*parameter*/, double /*value*/)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void PluginJoint::SetERP(double _newERP)
{
}

//////////////////////////////////////////////////
double PluginJoint::GetERP()
{
  return 0;
}

//////////////////////////////////////////////////
void PluginJoint::SetCFM(double _newCFM)
{
}

//////////////////////////////////////////////////
double PluginJoint::GetCFM()
{
  return 0;
}

//////////////////////////////////////////////////
bool PluginJoint::SetHighStop(unsigned int _index, const math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);
  switch (_index)
  {
    case 0:
      return true;
    case 1:
      return true;
    case 2:
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
bool PluginJoint::SetLowStop(unsigned int _index, const math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);
  switch (_index)
  {
    case 0:
      return true;
    case 1:
      return true;
    case 2:
      return true;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      return false;
  };
}

//////////////////////////////////////////////////
math::Angle PluginJoint::GetHighStop(unsigned int _index)
{
  return this->GetUpperLimit(_index);
}

//////////////////////////////////////////////////
math::Angle PluginJoint::GetLowStop(unsigned int _index)
{
  return this->GetLowerLimit(_index);
}

//////////////////////////////////////////////////
math::Vector3 PluginJoint::GetLinkForce(unsigned int _index) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Vector3 PluginJoint::GetLinkTorque(unsigned int _index) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  // record axis in sdf element
  if (_index == 0)
    this->sdf->GetElement("axis")->GetElement("xyz")->Set(_axis);
  else if (_index == 1)
    this->sdf->GetElement("axis2")->GetElement("xyz")->Set(_axis);
  else
    gzerr << "SetAxis index [" << _index << "] out of bounds\n";
}

//////////////////////////////////////////////////
bool PluginJoint::SetParam(const std::string &_key, unsigned int _index,
                            const boost::any &_value)
{
  if (_key == "fudge_factor")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "suspension_erp")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "suspension_cfm")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "stop_erp")
  {
    try
    {
      switch (_index)
      {
        case 0:
          break;
        case 1:
          break;
        case 2:
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "stop_cfm")
  {
    try
    {
      switch (_index)
      {
        case 0:
          break;
        case 1:
          break;
        case 2:
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "erp")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "cfm")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "fmax")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "vel")
  {
    try
    {
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "hi_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
          break;
        case 1:
          break;
        case 2:
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
          break;
        case 1:
          break;
        case 2:
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          return false;
      };
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else if (_key == "thread_pitch")
  {
    ScrewJoint<PluginJoint>* screwJoint =
      dynamic_cast<ScrewJoint<PluginJoint>* >(this);
    if (screwJoint != NULL)
    {
      try
      {
        screwJoint->SetThreadPitch(boost::any_cast<double>(_value));
      }
      catch(const boost::bad_any_cast &e)
      {
        gzerr << "boost any_cast error:" << e.what() << "\n";
        return false;
      }
    }
  }
  else if (_key == "gearbox_ratio")
  {
    GearboxJoint<PluginJoint>* gearboxJoint =
      dynamic_cast<GearboxJoint<PluginJoint>* >(this);
    if (gearboxJoint != NULL)
    {
      try
      {
        gearboxJoint->SetGearboxRatio(boost::any_cast<double>(_value));
      }
      catch(const boost::bad_any_cast &e)
      {
        gzerr << "boost any_cast error:" << e.what() << "\n";
        return false;
      }
    }
  }
  else
  {
    gzerr << "Unable to handle joint attribute[" << _key << "]\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
double PluginJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key == "fudge_factor")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "suspension_erp")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "suspension_cfm")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "stop_erp")
  {
    try
    {
      /// \TODO: switch based on index
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "stop_cfm")
  {
    try
    {
      /// \TODO: switch based on index
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "erp")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "cfm")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "fmax")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "vel")
  {
    try
    {
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "hi_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
        case 1:
        case 2:
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
        case 1:
        case 2:
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(common::Exception &e)
    {
      gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
      return 0;
    }
  }
  else if (_key == "thread_pitch")
  {
    ScrewJoint<PluginJoint>* screwJoint =
      dynamic_cast<ScrewJoint<PluginJoint>* >(this);
    if (screwJoint != NULL)
    {
      try
      {
        return screwJoint->GetThreadPitch();
      }
      catch(common::Exception &e)
      {
        gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
        return 0;
      }
    }
    else
    {
      gzerr << "Trying to get thread_pitch for non-screw joints.\n";
      return 0;
    }
  }
  else if (_key == "gearbox_ratio")
  {
    GearboxJoint<PluginJoint>* gearboxJoint =
      dynamic_cast<GearboxJoint<PluginJoint>* >(this);
    if (gearboxJoint != NULL)
    {
      try
      {
        return gearboxJoint->GetGearboxRatio();
      }
      catch(common::Exception &e)
      {
        gzerr << "GetParam error:" << e.GetErrorStr() << "\n";
        return 0;
      }
    }
    else
    {
      gzerr << "Trying to get thread_pitch for non-gearbox joints.\n";
      return 0;
    }
  }
  else
  {
    gzerr << "Unable to get joint attribute[" << _key << "]\n";
    return 0;
  }

  gzerr << "should not be here\n";
  return 0;
}

//////////////////////////////////////////////////
void PluginJoint::Reset()
{
  Joint::Reset();
}

//////////////////////////////////////////////////
JointWrench PluginJoint::GetForceTorque(unsigned int /*_index*/)
{
  return this->wrench;
}

//////////////////////////////////////////////////
bool PluginJoint::UsesImplicitSpringDamper()
{
  return this->useImplicitSpringDamper;
}

//////////////////////////////////////////////////
void PluginJoint::ApplyImplicitStiffnessDamping()
{
  // check if we are violating joint limits
  if (this->GetAngleCount() > 2)
  {
     gzerr << "Incompatible joint type, GetAngleCount() = "
           << this->GetAngleCount() << " > 2\n";
     return;
  }

  double dt = this->GetWorld()->GetPhysicsEngine()->GetMaxStepSize();
  for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
  {
    double angle = this->GetAngle(i).Radian();
    double dAngle = 2.0 * this->GetVelocity(i) * dt;
    angle += dAngle;

    if ((math::equal(this->dissipationCoefficient[i], 0.0) &&
         math::equal(this->stiffnessCoefficient[i], 0.0)) ||
        angle >= this->upperLimit[i].Radian() ||
        angle <= this->lowerLimit[i].Radian())
    {
      if (this->implicitDampingState[i] != PluginJoint::JOINT_LIMIT)
      {
        // We have hit the actual joint limit!
        // turn off simulated damping by recovering cfm and erp,
        // and recover joint limits
        this->SetParam("stop_erp", i, this->stopERP);
        this->SetParam("stop_cfm", i, this->stopCFM);
        this->SetParam("hi_stop", i, this->upperLimit[i].Radian());
        this->SetParam("lo_stop", i, this->lowerLimit[i].Radian());
        this->SetParam("hi_stop", i, this->upperLimit[i].Radian());
        this->implicitDampingState[i] = PluginJoint::JOINT_LIMIT;
      }
      /* test to see if we can reduce jitter at joint limits
      // apply spring damper explicitly if in joint limit
      // this limits oscillations if spring is pushing joint
      // into the limit.
      {
        double dampingForce = -fabs(this->dissipationCoefficient[i])
          * this->GetVelocity(i);
        double springForce = this->stiffnessCoefficient[i]
          * (this->springReferencePosition[i] - this->GetAngle(i).Radian());
        this->SetForceImpl(i, dampingForce + springForce);
      }
      */
    }
    else if (!math::equal(this->dissipationCoefficient[i], 0.0) ||
             !math::equal(this->stiffnessCoefficient[i], 0.0))
    {
      double kd = fabs(this->dissipationCoefficient[i]);
      double kp = this->stiffnessCoefficient[i];

      /// \TODO: This bit of code involving adaptive damping
      /// might be too complicated, add some more comments or simplify it.
      if (this->dissipationCoefficient[i] < 0)
        kd = this->ApplyAdaptiveDamping(i, kd);

      // update if going into DAMPING_ACTIVE mode, or
      // if current applied damping value is not the same as predicted.
      if (this->implicitDampingState[i] != PluginJoint::DAMPING_ACTIVE ||
          !math::equal(kd, this->currentKd[i]) ||
          !math::equal(kp, this->currentKp[i]))
      {
        // save kp, kd applied for efficiency
        this->currentKd[i] = kd;
        this->currentKp[i] = kp;

        // convert kp, kd to cfm, erp
        double erp, cfm;
        this->KpKdToCFMERP(dt, kp, kd, cfm, erp);

        // add additional constraint row by fake hitting joint limit
        // then, set erp and cfm to simulate viscous joint damping
        this->SetParam("stop_erp", i, erp);
        this->SetParam("stop_cfm", i, cfm);
        this->SetParam("hi_stop", i, this->springReferencePosition[i]);
        this->SetParam("lo_stop", i, this->springReferencePosition[i]);
        this->SetParam("hi_stop", i, this->springReferencePosition[i]);
        this->implicitDampingState[i] = PluginJoint::DAMPING_ACTIVE;
      }
    }
  }
}

//////////////////////////////////////////////////
double PluginJoint::ApplyAdaptiveDamping(unsigned int _index,
    const double _damping)
{
  /// \TODO: hardcoded thresholds for now, make them params.
  static double vThreshold = 0.01;
  static double fThreshold = 1.0;

  double f = this->GetForce(_index);
  double v = this->GetVelocity(_index);

  if (fabs(v) < vThreshold && fabs(f) > fThreshold)
  {
    // guess what the stable damping value might be based on v.
    double tmpDStable = f / (v/fabs(v)*std::max(fabs(v), vThreshold));

    // debug
    // gzerr << "joint [" << this->GetName()
    //       << "] damping[" << _damping
    //       << "] f [" << f
    //       << "] v [" << v
    //       << "] f*v [" << f*v
    //       << "] f/v [" << tmpDStable
    //       << "] cur currentKd[" << _index
    //       << "] = [" << currentKd[_index] << "]\n";

    // limit v(n+1)/v(n) to 2.0 by multiplying tmpDStable by 0.5
    return std::max(_damping, 0.5*tmpDStable);
  }
  else
    return _damping;
}

//////////////////////////////////////////////////
void PluginJoint::KpKdToCFMERP(const double _dt,
                           const double _kp, const double _kd,
                           double &_cfm, double &_erp)
{
  /// \TODO: check for NaN cases
  _erp = _dt * _kp / (_dt * _kp + _kd);
  _cfm = 1.0 / (_dt * _kp + _kd);
}

//////////////////////////////////////////////////
void PluginJoint::CFMERPToKpKd(const double _dt,
                           const double _cfm, const double _erp,
                           double &_kp, double &_kd)
{
  /// \TODO: check for NaN cases
  _kp = _erp / (_dt * _cfm);
  _kd = (1.0 - _erp) / _cfm;
}

//////////////////////////////////////////////////
void PluginJoint::SetDamping(unsigned int _index, double _damping)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index],
      _damping);
  }
  else
  {
     gzerr << "PluginJoint::SetDamping: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void PluginJoint::SetStiffness(unsigned int _index, double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->SetStiffnessDamping(_index, _stiffness,
      this->dissipationCoefficient[_index]);
  }
  else
  {
     gzerr << "PluginJoint::SetStiffness: index[" << _index
           << "] is out of bounds (GetAngleCount() = "
           << this->GetAngleCount() << ").\n";
     return;
  }
}

//////////////////////////////////////////////////
void PluginJoint::SetStiffnessDamping(unsigned int _index,
  double _stiffness, double _damping, double _reference)
{
  if (_index < this->GetAngleCount())
  {
    this->stiffnessCoefficient[_index] = _stiffness;
    this->dissipationCoefficient[_index] = _damping;
    this->springReferencePosition[_index] = _reference;

    /// reset state of implicit damping state machine.
    if (this->useImplicitSpringDamper)
    {
      if (static_cast<unsigned int>(_index) < this->GetAngleCount())
      {
        this->implicitDampingState[_index] = PluginJoint::NONE;
      }
      else
      {
         gzerr << "Incompatible joint type, index[" << _index
               << "] is out of bounds (GetAngleCount() = "
               << this->GetAngleCount() << ").\n";
         return;
      }
    }

    /// \TODO:  The check for static parent or child below might not be needed,
    /// but we need to test first.  In theory, attaching an object to a static
    /// body should not affect spring/damper application.
    bool parentStatic =
      this->GetParent() ? this->GetParent()->IsStatic() : false;
    bool childStatic =
      this->GetChild() ? this->GetChild()->IsStatic() : false;

    if (!this->stiffnessDampingInitialized)
    {
      if (!parentStatic && !childStatic)
      {
        this->applyDamping = physics::Joint::ConnectJointUpdate(
          boost::bind(&PluginJoint::ApplyStiffnessDamping, this));
        this->stiffnessDampingInitialized = true;
      }
      else
      {
        gzwarn << "Spring Damper for Joint[" << this->GetName()
               << "] is not initialized because either parent[" << parentStatic
               << "] or child[" << childStatic << "] is static.\n";
      }
    }
  }
  else
    gzerr << "SetStiffnessDamping _index too large.\n";
}

//////////////////////////////////////////////////
void PluginJoint::SetProvideFeedback(bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  if (this->provideFeedback)
  {
  }
}

//////////////////////////////////////////////////
void PluginJoint::SetForce(unsigned int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void PluginJoint::SaveForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->GetAngleCount())
  {
    if (this->forceAppliedTime < this->GetWorld()->GetSimTime())
    {
      // reset forces if time step is new
      this->forceAppliedTime = this->GetWorld()->GetSimTime();
      this->forceApplied[0] = this->forceApplied[1] = 0;
    }

    this->forceApplied[_index] += _force;
  }
  else
    gzerr << "Something's wrong, joint [" << this->GetScopedName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double PluginJoint::GetForce(unsigned int _index)
{
  if (_index < this->GetAngleCount())
  {
    return this->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void PluginJoint::ApplyStiffnessDamping()
{
  if (this->useImplicitSpringDamper)
    this->ApplyImplicitStiffnessDamping();
  else
    this->ApplyExplicitStiffnessDamping();
}

//////////////////////////////////////////////////
void PluginJoint::ApplyExplicitStiffnessDamping()
{
  for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
  {
    // Take absolute value of dissipationCoefficient, since negative values of
    // dissipationCoefficient are used for adaptive damping to
    // enforce stability.
    double dampingForce = -fabs(this->dissipationCoefficient[i])
      * this->GetVelocity(i);

    double springForce = this->stiffnessCoefficient[i]
      * (this->springReferencePosition[i] - this->GetAngle(i).Radian());

    // do not change forceApplied if setting internal damping forces
    this->SetForceImpl(i, dampingForce + springForce);

    // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
  }
}

//////////////////////////////////////////////////
bool PluginJoint::SetPosition(unsigned int _index, double _position)
{
  return Joint::SetPositionMaximal(_index, _position);
}
