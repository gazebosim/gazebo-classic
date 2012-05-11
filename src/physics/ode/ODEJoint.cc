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
/* Desc: The ODE base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 12 Oct 2009
 */

#include "common/Exception.hh"
#include "common/Console.hh"

#include "physics/World.hh"
#include "physics/Body.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/ode/ODEBody.hh"
#include "physics/ode/ODEJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODEJoint::ODEJoint()
  : Joint()
{
  this->jointId = NULL;
}

//////////////////////////////////////////////////
ODEJoint::~ODEJoint()
{
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

    if (elem->HasElement("limit"))
    {
      this->SetParam(dParamStopERP,
          elem->GetElement("limit")->GetValueDouble("erp"));
      this->SetParam(dParamStopCFM,
          elem->GetElement("limit")->GetValueDouble("cfm"));
    }

    if (elem->HasElement("suspension"))
    {
      this->SetParam(dParamSuspensionERP,
          elem->GetElement("suspension")->GetValueDouble("erp"));
      this->SetParam(dParamSuspensionCFM,
          elem->GetElement("suspension")->GetValueDouble("cfm"));
    }

    if (elem->HasElement("fudge_factor"))
      this->SetParam(dParamFudgeFactor,
          elem->GetElement("fudge_factor")->GetValueDouble());

    if (elem->HasElement("cfm"))
        this->SetParam(dParamCFM, elem->GetElement("cfm")->GetValueDouble());

    if (elem->HasElement("bounce"))
        this->SetParam(dParamBounce,
          elem->GetElement("bounce")->GetValueDouble());

    if (elem->HasElement("max_force"))
      this->SetParam(dParamFMax,
          elem->GetElement("max_force")->GetValueDouble());

    if (elem->HasElement("velocity"))
      this->SetParam(dParamVel,
          elem->GetElement("velocity")->GetValueDouble());
  }

  // TODO: reimplement
  /*if (**this->provideFeedbackP)
  {
    this->feedback = new dJointFeedback;
    dJointSetFeedback(this->jointId, this->feedback);
  }
  */
}

//////////////////////////////////////////////////
BodyPtr ODEJoint::GetJointBody(int _index) const
{
  BodyPtr result;

  if (_index == 0 || _index == 1)
  {
    ODEBodyPtr odeBody1 = boost::shared_static_cast<ODEBody>(this->childBody);
    ODEBodyPtr odeBody2 = boost::shared_static_cast<ODEBody>(this->parentBody);
    if (odeBody1 != NULL &&
        dJointGetBody(this->jointId, _index) == odeBody1->GetODEId())
      result = this->childBody;
    else if (odeBody2)
      result = this->parentBody;
  }

  return result;
}

//////////////////////////////////////////////////
bool ODEJoint::AreConnected(BodyPtr _one, BodyPtr _two) const
{
  ODEBodyPtr odeBody1 = boost::shared_dynamic_cast<ODEBody>(_one);
  ODEBodyPtr odeBody2 = boost::shared_dynamic_cast<ODEBody>(_two);

  if (odeBody1 == NULL || odeBody2 == NULL)
    gzthrow("ODEJoint requires ODE bodies\n");

  return dAreConnected(odeBody1->GetODEId(), odeBody2->GetODEId());
}

//////////////////////////////////////////////////
// child classes where appropriate
double ODEJoint::GetParam(int /*parameter*/) const
{
  return 0;
}

//////////////////////////////////////////////////
void ODEJoint::Attach(BodyPtr _parent, BodyPtr _child)
{
  Joint::Attach(_parent, _child);

  ODEBodyPtr odechild = boost::shared_dynamic_cast<ODEBody>(this->childBody);
  ODEBodyPtr odeparent = boost::shared_dynamic_cast<ODEBody>(this->parentBody);

  if (odechild == NULL && odeparent == NULL)
    gzthrow("ODEJoint requires at least one ODE body\n");


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
  this->childBody.reset();
  this->parentBody.reset();
  dJointAttach(this->jointId, 0, 0);
}

//////////////////////////////////////////////////
// where appropriate
void ODEJoint::SetParam(int /*parameter*/, double /*value*/)
{
  if (this->childBody)
    this->childBody->SetEnabled(true);
  if (this->parentBody)
    this->parentBody->SetEnabled(true);
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
  return dJointGetFeedback(this->jointId);
}

//////////////////////////////////////////////////
void ODEJoint::SetHighStop(int _index, math::Angle _angle)
{
  switch (_index)
  {
    case 0:
      this->SetParam(dParamHiStop, _angle.GetAsRadian());
      break;
    case 1:
      this->SetParam(dParamHiStop2, _angle.GetAsRadian());
      break;
    case 2:
      this->SetParam(dParamHiStop3, _angle.GetAsRadian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
      break;
  };
}

//////////////////////////////////////////////////
void ODEJoint::SetLowStop(int _index, math::Angle _angle)
{
  switch (_index)
  {
    case 0:
      this->SetParam(dParamLoStop, _angle.GetAsRadian());
      break;
    case 1:
      this->SetParam(dParamLoStop2, _angle.GetAsRadian());
      break;
    case 2:
      this->SetParam(dParamLoStop3, _angle.GetAsRadian());
      break;
    default:
      gzerr << "Invalid index[" << _index << "]\n";
  };
}

//////////////////////////////////////////////////
math::Angle ODEJoint::GetHighStop(int _index)
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
  };

  return 0;
}

//////////////////////////////////////////////////
math::Angle ODEJoint::GetLowStop(int _index)
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
  };

  return 0;
}

//////////////////////////////////////////////////
math::Vector3 ODEJoint::GetBodyForce(unsigned int _index) const
{
  math::Vector3 result;
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
math::Vector3 ODEJoint::GetBodyTorque(unsigned int _index) const
{
  math::Vector3 result;
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
void ODEJoint::SetAttribute(Attribute _attr, int /*_index*/, double _value)
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
      this->SetParam(dParamStopERP, _value);
      break;
    case STOP_CFM:
      this->SetParam(dParamStopCFM, _value);
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
      this->SetParam(dParamHiStop, _value);
      break;
    case LO_STOP:
      this->SetParam(dParamLoStop, _value);
      break;
    default:
      gzerr << "Unable to handle joint attribute[" << _attr << "]\n";
      break;
  };
}

void ODEJoint::Reset()
{
  dJointReset(this->jointId);
  Joint::Reset();
}
