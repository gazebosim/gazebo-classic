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
/* Desc: The ODE base joint class
 * Author: Nate Koenig, Andrew Howard
 * Date: 12 Oct 2009
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEJoint.hh"
#include "gazebo/physics/ScrewJoint.hh"
#include "gazebo/physics/JointWrench.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEJoint::ODEJoint(BasePtr _parent)
  : Joint(_parent)
{
  this->jointId = NULL;
  this->cfmDampingState[0] = ODEJoint::NONE;
  this->cfmDampingState[1] = ODEJoint::NONE;
  this->cfmDampingState[2] = ODEJoint::NONE;
  this->dampingInitialized = false;
  this->feedback = NULL;
  this->dStable[0] = 0;
  this->dStable[1] = 0;
  this->dStable[2] = 0;
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
}

//////////////////////////////////////////////////
ODEJoint::~ODEJoint()
{
  delete this->feedback;
  this->Detach();

  if (this->jointId)
    dJointDestroy(this->jointId);
}

//////////////////////////////////////////////////
void ODEJoint::Load(sdf::ElementPtr _sdf)
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("physics") &&
      this->sdf->GetElement("physics")->HasElement("ode"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("physics")->GetElement("ode");

    if (elem->HasElement("cfm_damping"))
    {
      this->useCFMDamping = elem->Get<bool>("cfm_damping");
    }

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopERP = elem->GetElement("limit")->Get<double>("erp");
    for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
      this->SetAttribute("stop_erp", i, this->stopERP);

    // initializa both axis, \todo: make cfm, erp per axis
    this->stopCFM = elem->GetElement("limit")->Get<double>("cfm");
    for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
      this->SetAttribute("stop_cfm", i, this->stopCFM);

    if (elem->HasElement("suspension"))
    {
      this->SetParam(dParamSuspensionERP,
          elem->GetElement("suspension")->Get<double>("erp"));
      this->SetParam(dParamSuspensionCFM,
          elem->GetElement("suspension")->Get<double>("cfm"));
    }

    if (elem->HasElement("fudge_factor"))
      this->SetParam(dParamFudgeFactor,
          elem->GetElement("fudge_factor")->Get<double>());

    if (elem->HasElement("cfm"))
        this->SetAttribute("cfm", 0, elem->Get<double>("cfm"));

    if (elem->HasElement("erp"))
        this->SetAttribute("erp", 0, elem->Get<double>("erp"));

    if (elem->HasElement("bounce"))
        this->SetParam(dParamBounce,
          elem->GetElement("bounce")->Get<double>());

    if (elem->HasElement("max_force"))
      this->SetParam(dParamFMax,
          elem->GetElement("max_force")->Get<double>());

    if (elem->HasElement("velocity"))
      this->SetParam(dParamVel,
          elem->GetElement("velocity")->Get<double>());
  }

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    if (axisElem->HasElement("dynamics"))
    {
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");

      if (dynamicsElem->HasElement("damping"))
      {
        this->SetDamping(0, dynamicsElem->Get<double>("damping"));
      }
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

      if (dynamicsElem->HasElement("damping"))
      {
        this->SetDamping(1, dynamicsElem->Get<double>("damping"));
      }
      if (dynamicsElem->HasElement("friction"))
      {
        sdf::ElementPtr frictionElem = dynamicsElem->GetElement("friction");
        gzlog << "joint friction not implemented\n";
      }
    }
  }
}

//////////////////////////////////////////////////
LinkPtr ODEJoint::GetJointLink(int _index) const
{
  LinkPtr result;
  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  if (_index == 0 || _index == 1)
  {
    ODELinkPtr odeLink1 = boost::static_pointer_cast<ODELink>(this->childLink);
    ODELinkPtr odeLink2 = boost::static_pointer_cast<ODELink>(this->parentLink);
    if (odeLink1 != NULL &&
        dJointGetBody(this->jointId, _index) == odeLink1->GetODEId())
      result = this->childLink;
    else if (odeLink2)
      result = this->parentLink;
  }

  return result;
}

//////////////////////////////////////////////////
bool ODEJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  ODELinkPtr odeLink1 = boost::dynamic_pointer_cast<ODELink>(_one);
  ODELinkPtr odeLink2 = boost::dynamic_pointer_cast<ODELink>(_two);

  if (odeLink1 == NULL || odeLink2 == NULL)
    gzthrow("ODEJoint requires ODE bodies\n");

  return dAreConnected(odeLink1->GetODEId(), odeLink2->GetODEId());
}

//////////////////////////////////////////////////
// child classes where appropriate
double ODEJoint::GetParam(int /*parameter*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  Joint::Attach(_parent, _child);

  ODELinkPtr odechild = boost::dynamic_pointer_cast<ODELink>(this->childLink);
  ODELinkPtr odeparent = boost::dynamic_pointer_cast<ODELink>(this->parentLink);

  if (odechild == NULL && odeparent == NULL)
    gzthrow("ODEJoint requires at least one ODE link\n");

  if (!this->jointId)
    gzerr << "ODE Joint ID is invalid\n";

  if (this->HasType(Base::HINGE2_JOINT) &&
      (odechild == NULL || odeparent == NULL))
    gzthrow("ODEHinge2Joint cannot be connected to the world");

  if (!odechild && odeparent)
  {
    dJointAttach(this->jointId, 0, odeparent->GetODEId());
  }
  else if (odechild && !odeparent)
  {
    dJointAttach(this->jointId, odechild->GetODEId(), 0);
  }
  else if (odechild && odeparent)
  {
    if (this->HasType(Base::HINGE2_JOINT))
      dJointAttach(this->jointId, odeparent->GetODEId(), odechild->GetODEId());
    else
      dJointAttach(this->jointId, odechild->GetODEId(), odeparent->GetODEId());
  }
}

//////////////////////////////////////////////////
void ODEJoint::Detach()
{
  Joint::Detach();
  this->childLink.reset();
  this->parentLink.reset();

  if (this->jointId)
    dJointAttach(this->jointId, 0, 0);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
// where appropriate
void ODEJoint::SetParam(int /*parameter*/, double /*value*/)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SetERP(double _newERP)
{
  this->SetParam(dParamSuspensionERP, _newERP);
}

//////////////////////////////////////////////////
double ODEJoint::GetERP()
{
  return this->GetParam(dParamSuspensionERP);
}

//////////////////////////////////////////////////
void ODEJoint::SetCFM(double _newCFM)
{
  this->SetParam(dParamSuspensionCFM, _newCFM);
}

//////////////////////////////////////////////////
double ODEJoint::GetCFM()
{
  return this->GetParam(dParamSuspensionCFM);
}

//////////////////////////////////////////////////
dJointFeedback *ODEJoint::GetFeedback()
{
  if (this->jointId)
    return dJointGetFeedback(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";
  return NULL;
}

//////////////////////////////////////////////////
void ODEJoint::SetHighStop(int _index, const math::Angle &_angle)
{
  Joint::SetHighStop(_index, _angle);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamHiStop, _angle.Radian());
      break;
    case 1:
      this->SetParam(dParamHiStop2, _angle.Radian());
      break;
    case 2:
      this->SetParam(dParamHiStop3, _angle.Radian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      break;
  };
}

//////////////////////////////////////////////////
void ODEJoint::SetLowStop(int _index, const math::Angle &_angle)
{
  Joint::SetLowStop(_index, _angle);
  switch (_index)
  {
    case 0:
      this->SetParam(dParamLoStop, _angle.Radian());
      break;
    case 1:
      this->SetParam(dParamLoStop2, _angle.Radian());
      break;
    case 2:
      this->SetParam(dParamLoStop3, _angle.Radian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
math::Angle ODEJoint::GetHighStop(int _index)
{
  return this->GetUpperLimit(_index);
}

//////////////////////////////////////////////////
math::Angle ODEJoint::GetLowStop(int _index)
{
  return this->GetLowerLimit(_index);
}

//////////////////////////////////////////////////
math::Vector3 ODEJoint::GetLinkForce(unsigned int _index) const
{
  math::Vector3 result;

  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);

  if (_index == 0)
    result.Set(jointFeedback->f1[0], jointFeedback->f1[1],
               jointFeedback->f1[2]);
  else
    result.Set(jointFeedback->f2[0], jointFeedback->f2[1],
               jointFeedback->f2[2]);

  return result;
}

//////////////////////////////////////////////////
math::Vector3 ODEJoint::GetLinkTorque(unsigned int _index) const
{
  math::Vector3 result;

  if (!this->jointId)
  {
    gzerr << "ODE Joint ID is invalid\n";
    return result;
  }

  dJointFeedback *jointFeedback = dJointGetFeedback(this->jointId);

  if (_index == 0)
    result.Set(jointFeedback->t1[0], jointFeedback->t1[1],
               jointFeedback->t1[2]);
  else
    result.Set(jointFeedback->t2[0], jointFeedback->t2[1],
               jointFeedback->t2[2]);

  return result;
}

//////////////////////////////////////////////////
void ODEJoint::SetAxis(int _index, const math::Vector3 &_axis)
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
void ODEJoint::SetAttribute(Attribute _attr, int _index, double _value)
{
  switch (_attr)
  {
    case FUDGE_FACTOR:
      this->SetParam(dParamFudgeFactor, _value);
      break;
    case SUSPENSION_ERP:
      this->SetParam(dParamSuspensionERP, _value);
      break;
    case SUSPENSION_CFM:
      this->SetParam(dParamSuspensionCFM, _value);
      break;
    case STOP_ERP:
      switch (_index)
      {
        case 0:
          this->SetParam(dParamStopERP, _value);
          break;
        case 1:
          this->SetParam(dParamStopERP2, _value);
          break;
        case 2:
          this->SetParam(dParamStopERP3, _value);
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
      break;
    case STOP_CFM:
      switch (_index)
      {
        case 0:
          this->SetParam(dParamStopCFM, _value);
          break;
        case 1:
          this->SetParam(dParamStopCFM2, _value);
          break;
        case 2:
          this->SetParam(dParamStopCFM3, _value);
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
      break;
    case ERP:
      this->SetParam(dParamERP, _value);
      break;
    case CFM:
      this->SetParam(dParamCFM, _value);
      break;
    case FMAX:
      this->SetParam(dParamFMax, _value);
      break;
    case VEL:
      this->SetParam(dParamVel, _value);
      break;
    case HI_STOP:
      switch (_index)
      {
        case 0:
          this->SetParam(dParamHiStop, _value);
          break;
        case 1:
          this->SetParam(dParamHiStop2, _value);
          break;
        case 2:
          this->SetParam(dParamHiStop3, _value);
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
      break;
    case LO_STOP:
      switch (_index)
      {
        case 0:
          this->SetParam(dParamLoStop, _value);
          break;
        case 1:
          this->SetParam(dParamLoStop2, _value);
          break;
        case 2:
          this->SetParam(dParamLoStop3, _value);
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
      break;
    default:
      gzerr << "Unable to handle joint attribute[" << _attr << "]\n";
      break;
  };
}

//////////////////////////////////////////////////
void ODEJoint::SetAttribute(const std::string &_key, int _index,
                            const boost::any &_value)
{
  if (_key == "fudge_factor")
  {
    try
    {
      this->SetParam(dParamFudgeFactor, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "suspension_erp")
  {
    try
    {
      this->SetParam(dParamSuspensionERP, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "suspension_cfm")
  {
    try
    {
      this->SetParam(dParamSuspensionCFM, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "stop_erp")
  {
    try
    {
      switch (_index)
      {
        case 0:
          this->SetParam(dParamStopERP, boost::any_cast<double>(_value));
          break;
        case 1:
          this->SetParam(dParamStopERP2, boost::any_cast<double>(_value));
          break;
        case 2:
          this->SetParam(dParamStopERP3, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "stop_cfm")
  {
    try
    {
      switch (_index)
      {
        case 0:
          this->SetParam(dParamStopCFM, boost::any_cast<double>(_value));
          break;
        case 1:
          this->SetParam(dParamStopCFM2, boost::any_cast<double>(_value));
          break;
        case 2:
          this->SetParam(dParamStopCFM3, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "erp")
  {
    try
    {
      this->SetParam(dParamERP, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "cfm")
  {
    try
    {
      this->SetParam(dParamCFM, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "fmax")
  {
    try
    {
      this->SetParam(dParamFMax, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "vel")
  {
    try
    {
      this->SetParam(dParamVel, boost::any_cast<double>(_value));
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "hi_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
          this->SetParam(dParamHiStop, boost::any_cast<double>(_value));
          break;
        case 1:
          this->SetParam(dParamHiStop2, boost::any_cast<double>(_value));
          break;
        case 2:
          this->SetParam(dParamHiStop3, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "lo_stop")
  {
    try
    {
      switch (_index)
      {
        case 0:
          this->SetParam(dParamLoStop, boost::any_cast<double>(_value));
          break;
        case 1:
          this->SetParam(dParamLoStop2, boost::any_cast<double>(_value));
          break;
        case 2:
          this->SetParam(dParamLoStop3, boost::any_cast<double>(_value));
          break;
        default:
          gzerr << "Invalid index[" << _index << "]\n";
          break;
      };
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
  else if (_key == "thread_pitch")
  {
    ScrewJoint<ODEJoint>* screwJoint =
      dynamic_cast<ScrewJoint<ODEJoint>* >(this);
    if (screwJoint != NULL)
    {
      try
      {
        screwJoint->SetThreadPitch(0, boost::any_cast<double>(_value));
      }
      catch(boost::bad_any_cast &e)
      {
        gzerr << "boost any_cast error:" << e.what() << "\n";
      }
    }
  }
  else
  {
    try
    {
      gzerr << "Unable to handle joint attribute["
            << boost::any_cast<std::string>(_value) << "]\n";
    }
    catch(boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
    }
  }
}

//////////////////////////////////////////////////
double ODEJoint::GetAttribute(const std::string &_key, unsigned int _index)
{
  if (_key == "fudge_factor")
  {
    try
    {
      return this->GetParam(dParamFudgeFactor);
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
      return this->GetParam(dParamSuspensionERP);
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
      return this->GetParam(dParamSuspensionCFM);
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
      return this->GetParam(dParamStopERP);
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
      return this->GetParam(dParamStopCFM);
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
      return this->GetParam(dParamERP);
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
      return this->GetParam(dParamCFM);
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
      return this->GetParam(dParamFMax);
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
      return this->GetParam(dParamVel);
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
          return this->GetParam(dParamHiStop);
        case 1:
          return this->GetParam(dParamHiStop2);
        case 2:
          return this->GetParam(dParamHiStop3);
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
          return this->GetParam(dParamLoStop);
        case 1:
          return this->GetParam(dParamLoStop2);
        case 2:
          return this->GetParam(dParamLoStop3);
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
    ScrewJoint<ODEJoint>* screwJoint =
      dynamic_cast<ScrewJoint<ODEJoint>* >(this);
    if (screwJoint != NULL)
    {
      try
      {
        return screwJoint->GetThreadPitch(0);
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
  else
  {
    gzerr << "Unable to get joint attribute[" << _key << "]\n";
    return 0;
  }

  gzerr << "should not be here\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Reset()
{
  if (this->jointId)
    dJointReset(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  Joint::Reset();
}

//////////////////////////////////////////////////
JointWrench ODEJoint::GetForceTorque(unsigned int /*_index*/)
{
  // Note that:
  // f2, t2 are the force torque measured on parent body's cg
  // f1, t1 are the force torque measured on child body's cg
  dJointFeedback *fb = this->GetFeedback();
  if (fb)
  {
    // kind of backwards here, body1 (parent) corresponds go f2, t2
    // and body2 (child) corresponds go f1, t1
    this->wrench.body2Force.Set(fb->f1[0], fb->f1[1], fb->f1[2]);
    this->wrench.body2Torque.Set(fb->t1[0], fb->t1[1], fb->t1[2]);
    this->wrench.body1Force.Set(fb->f2[0], fb->f2[1], fb->f2[2]);
    this->wrench.body1Torque.Set(fb->t2[0], fb->t2[1], fb->t2[2]);

    // get force applied through SetForce
    physics::JointWrench wrenchAppliedWorld;
    if (this->HasType(physics::Base::HINGE_JOINT))
    {
      // rotate force into child link frame
      // GetLocalAxis is the axis specified in parent link frame!!!
      wrenchAppliedWorld.body2Torque =
        this->GetForce(0u) * this->GetLocalAxis(0u);

      // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
      //       << "] axis [" << this->GetLocalAxis(0u)
      //       << "]\n";

      wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
    }
    else if (this->HasType(physics::Base::SLIDER_JOINT))
    {
      // rotate force into child link frame
      wrenchAppliedWorld.body2Force =
        this->GetForce(0u) * this->GetLocalAxis(0u);
      wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
    }
    else
    {
      /// \TODO: fix for multi-axis joints
      gzerr << "force torque for joint type [" << this->GetType()
            << "] not implemented, returns false results!!\n";
    }

    // convert wrench from child cg location to child link frame
    if (this->childLink)
    {
      math::Pose childPose = this->childLink->GetWorldPose();

      // convert torque from about child CG to joint anchor location
      // cg position specified in child link frame
      math::Pose cgPose = this->childLink->GetInertial()->GetPose();

      // anchorPose location of joint in child frame
      // childMomentArm: from child CG to joint location in child link frame
      // moment arm rotated into world frame (given feedback is in world frame)
      math::Vector3 childMomentArm = childPose.rot.RotateVector(
        (this->anchorPose - math::Pose(cgPose.pos, math::Quaternion())).pos);

      // gzerr << "anchor [" << anchorPose
      //       << "] iarm[" << this->childLink->GetInertial()->GetPose().pos
      //       << "] childMomentArm[" << childMomentArm
      //       << "] f1[" << this->wrench.body2Force
      //       << "] t1[" << this->wrench.body2Torque
      //       << "] fxp[" << this->wrench.body2Force.Cross(childMomentArm)
      //       << "]\n";

      this->wrench.body2Torque += this->wrench.body2Force.Cross(childMomentArm);

      // rotate resulting body2Force in world frame into link frame
      this->wrench.body2Force = childPose.rot.RotateVectorReverse(
        -this->wrench.body2Force);

      // rotate resulting body2Torque in world frame into link frame
      this->wrench.body2Torque = childPose.rot.RotateVectorReverse(
        -this->wrench.body2Torque);
    }

    // convert torque from about parent CG to joint anchor location
    if (this->parentLink)
    {
      // get child pose, or it's the inertial world if childLink is NULL
      math::Pose childPose;
      if (this->childLink)
        childPose = this->childLink->GetWorldPose();
      else
        gzerr << "missing child link, double check model.";

      math::Pose parentPose = this->parentLink->GetWorldPose();

      // if parent link exists, convert torque from about parent
      // CG to joint anchor location

      // parent cg specified in parent link frame
      math::Pose cgPose = this->parentLink->GetInertial()->GetPose();

      // get parent CG pose in child link frame
      math::Pose parentCGInChildLink =
        math::Pose(cgPose.pos, math::Quaternion()) - (childPose - parentPose);

      // anchor location in parent CG frame
      // this is the moment arm, but it's in parent CG frame, we need
      // to convert it into world frame
      math::Pose anchorInParendCGFrame = this->anchorPose - parentCGInChildLink;

      // paretnCGFrame in world frame
      math::Pose parentCGInWorld = cgPose + parentPose;

      // rotate momeent arms into world frame
      math::Vector3 parentMomentArm = parentCGInWorld.rot.RotateVector(
        (this->anchorPose - parentCGInChildLink).pos);

      // gzerr << "anchor [" << this->anchorPose
      //       << "] pcginc[" << parentCGInChildLink
      //       << "] iarm[" << cgPose
      //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
      //       << "] parentMomentArm[" << parentMomentArm
      //       << "] f1[" << this->wrench.body1Force
      //       << "] t1[" << this->wrench.body1Torque
      //       << "] fxp[" << this->wrench.body1Force.Cross(parentMomentArm)
      //       << "]\n";

      this->wrench.body1Torque +=
        this->wrench.body1Force.Cross(parentMomentArm);

      // rotate resulting body1Force in world frame into link frame
      this->wrench.body1Force = parentPose.rot.RotateVectorReverse(
        -this->wrench.body1Force);

      // rotate resulting body1Torque in world frame into link frame
      this->wrench.body1Torque = parentPose.rot.RotateVectorReverse(
        -this->wrench.body1Torque);

      if (!this->childLink)
      {
        gzlog << "Joint [" << this->GetScopedName()
              << "] with parent Link [" << this->parentLink->GetScopedName()
              << "] but no child Link.  Child Link must be world.\n";
        // if child link does not exist, use equal and opposite
        this->wrench.body2Force = -this->wrench.body1Force;
        this->wrench.body2Torque = -this->wrench.body1Torque;

        // force/torque are in parent link frame, transform them into
        // child link(world) frame.
        math::Pose parentToWorldTransform = this->parentLink->GetWorldPose();
        this->wrench.body1Force =
          parentToWorldTransform.rot.RotateVector(
          this->wrench.body1Force);
        this->wrench.body1Torque =
          parentToWorldTransform.rot.RotateVector(
          this->wrench.body1Torque);
      }
    }
    else
    {
      if (!this->childLink)
      {
        gzerr << "Both parent and child links are invalid, abort.\n";
        return JointWrench();
      }
      else
      {
        gzlog << "Joint [" << this->GetScopedName()
              << "] with child Link [" << this->childLink->GetScopedName()
              << "] but no parent Link.  Parent Link must be world.\n";
        // if parentLink does not exist, use equal opposite body1 wrench
        this->wrench.body1Force = -this->wrench.body2Force;
        this->wrench.body1Torque = -this->wrench.body2Torque;

        // force/torque are in child link frame, transform them into
        // parent link frame.  Here, parent link is world, so zero transform.
        math::Pose childToWorldTransform = this->childLink->GetWorldPose();
        this->wrench.body1Force =
          childToWorldTransform.rot.RotateVector(
          this->wrench.body1Force);
        this->wrench.body1Torque =
          childToWorldTransform.rot.RotateVector(
          this->wrench.body1Torque);
      }
    }
    this->wrench = this->wrench - wrenchAppliedWorld;
  }
  else
  {
    // forgot to set provide_feedback?
    gzwarn << "GetForceTorque: forgot to set <provide_feedback>?\n";
  }

  return this->wrench;
}

//////////////////////////////////////////////////
void ODEJoint::CFMDamping()
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
    if (math::equal(this->dampingCoefficient, 0.0) ||
        angle + dAngle >= this->upperLimit[i].Radian() ||
        angle + dAngle <= this->lowerLimit[i].Radian())
    {
      if (this->cfmDampingState[i] != ODEJoint::JOINT_LIMIT)
      {
        // we have hit the actual joint limit!
        // turn off simulated damping by recovering cfm and erp,
        // and recover joint limits
        this->SetAttribute("stop_erp", i, this->stopERP);
        this->SetAttribute("stop_cfm", i, this->stopCFM);
        this->SetAttribute("hi_stop", i, this->upperLimit[i].Radian());
        this->SetAttribute("lo_stop", i, this->lowerLimit[i].Radian());
        this->SetAttribute("hi_stop", i, this->upperLimit[i].Radian());
        this->cfmDampingState[i] = ODEJoint::JOINT_LIMIT;
      }
    }
    else if (!math::equal(this->dampingCoefficient, 0.0))
    {
      /// \TODO: hardcoded thresholds for now, make them params.
      static double vThreshold = 0.01;
      static double fThreshold = 1.0;
      double f = this->GetForce(i);
      double v = this->GetVelocity(i);
      double curDamp = fabs(this->dampingCoefficient);

      /// \TODO: increase damping if resulting acceleration
      /// is outside of stability region.  simple limiter based on (f, v).
      // safeguard against unstable joint behavior
      // at low speed and high force scenarios
      if (this->dampingCoefficient < 0 &&
          fabs(v) < vThreshold && fabs(f) > fThreshold)
      {
        double tmpDStable = f / (v/fabs(v)*std::max(fabs(v), vThreshold));
        // gzerr << "joint [" << this->GetName()
        //       << "] curDamp[" << curDamp
        //       << "] f [" << f
        //       << "] v [" << v
        //       << "] f*v [" << f*v
        //       << "] f/v [" << tmpDStable
        //       << "] cur dStable[" << i << "] = [" << dStable[i] << "]\n";

        // limit v(n+1)/v(n) to 2.0 by multiplying tmpDStable by 0.5
        curDamp = std::max(curDamp, 0.5*tmpDStable);
      }

      // update if going into DAMPING_ACTIVE mode, or
      // if current applied damping value is not the same as predicted.
      if (this->cfmDampingState[i] != ODEJoint::DAMPING_ACTIVE ||
          !math::equal(curDamp, this->dStable[i]))
      {
        this->dStable[i] = curDamp;
        // add additional constraint row by fake hitting joint limit
        // then, set erp and cfm to simulate viscous joint damping
        this->SetAttribute("stop_erp", i, 0.0);
        this->SetAttribute("stop_cfm", i, 1.0 / curDamp);
        this->SetAttribute("hi_stop", i, 0.0);
        this->SetAttribute("lo_stop", i, 0.0);
        this->SetAttribute("hi_stop", i, 0.0);
        this->cfmDampingState[i] = ODEJoint::DAMPING_ACTIVE;
      }
    }
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetDamping(int /*_index*/, double _damping)
{
  this->dampingCoefficient = _damping;

  // \TODO: implement on a per axis basis (requires additional sdf parameters)
  // trigger an update in CFMDAmping if this is called
  if (this->useCFMDamping)
  {
    if (this->GetAngleCount() > 2)
    {
       gzerr << "Incompatible joint type, GetAngleCount() = "
             << this->GetAngleCount() << " > 2\n";
       return;
    }
    for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
      this->cfmDampingState[i] = ODEJoint::NONE;
  }

  /// \TODO:  this check might not be needed?  attaching an object to a static
  /// body should not affect damping application.
  bool parentStatic = this->GetParent() ? this->GetParent()->IsStatic() : false;
  bool childStatic = this->GetChild() ? this->GetChild()->IsStatic() : false;

  if (!this->dampingInitialized && !parentStatic && !childStatic)
  {
    if (this->useCFMDamping)
      this->applyDamping = physics::Joint::ConnectJointUpdate(
        boost::bind(&ODEJoint::CFMDamping, this));
    else
      this->applyDamping = physics::Joint::ConnectJointUpdate(
        boost::bind(&ODEJoint::ApplyDamping, this));
    this->dampingInitialized = true;
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetProvideFeedback(bool _enable)
{
  Joint::SetProvideFeedback(_enable);

  if (this->provideFeedback)
  {
    this->feedback = new dJointFeedback;
    this->feedback->f1[0] = 0;
    this->feedback->f1[1] = 0;
    this->feedback->f1[2] = 0;
    this->feedback->t1[0] = 0;
    this->feedback->t1[1] = 0;
    this->feedback->t1[2] = 0;
    this->feedback->f2[0] = 0;
    this->feedback->f2[1] = 0;
    this->feedback->f2[2] = 0;
    this->feedback->t2[0] = 0;
    this->feedback->t2[1] = 0;
    this->feedback->t2[2] = 0;

    if (this->jointId)
      dJointSetFeedback(this->jointId, this->feedback);
    else
      gzerr << "ODE Joint ID is invalid\n";
  }
}

//////////////////////////////////////////////////
void ODEJoint::SetForce(int _index, double _force)
{
  double force = Joint::CheckAndTruncateForce(_index, _force);
  this->SaveForce(_index, force);
  this->SetForceImpl(_index, force);

  // for engines that supports auto-disable of links
  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

//////////////////////////////////////////////////
void ODEJoint::SaveForce(int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index >= 0 && static_cast<unsigned int>(_index) < this->GetAngleCount())
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
double ODEJoint::GetForce(unsigned int _index)
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
void ODEJoint::ApplyDamping()
{
  // Take absolute value of dampingCoefficient, since negative values of
  // dampingCoefficient are used for adaptive damping to enforce stability.
  double dampingForce = -fabs(this->dampingCoefficient) * this->GetVelocity(0);

  // do not change forceApplied if setting internal damping forces
  this->SetForceImpl(0, dampingForce);

  // gzerr << this->GetVelocity(0) << " : " << dampingForce << "\n";
}
