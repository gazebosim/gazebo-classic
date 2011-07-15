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
 * SVN: $Id$
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


////////////////////////////////////////////////////////////////////////////////
/// Constructor
ODEJoint::ODEJoint()
  : Joint()
{
  this->jointId = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
ODEJoint::~ODEJoint()
{
  dJointDestroy( this->jointId );
}

////////////////////////////////////////////////////////////////////////////////
/// Load a joint
void ODEJoint::Load( sdf::ElementPtr &_sdf )
{
  Joint::Load(_sdf);

  if (this->sdf->HasElement("physics") && 
      this->sdf->GetElement("physics")->HasElement("ode"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("physics")->GetElement("ode");

    if (elem->HasElement("limit"))
    {
      this->SetParam(dParamStopERP, 
          elem->GetElement("limit")->GetValueDouble("cfm"));
      this->SetParam(dParamStopCFM, 
          elem->GetElement("limit")->GetValueDouble("erp"));
    }

    if (elem->HasElement("suspension"))
    {
      this->SetParam(dParamSuspensionERP, 
          elem->GetElement("suspension")->GetValueDouble("cfm"));
      this->SetParam(dParamSuspensionCFM, 
          elem->GetElement("suspension")->GetValueDouble("erp"));
    }

    if (elem->HasElement("fudge_factor"))
      this->SetParam(dParamFudgeFactor, 
          elem->GetElement("fudge_factor")->GetValueDouble());

    if (elem->HasElement("cfm"))
        this->SetParam(dParamCFM, 
          elem->GetElement("cfm")->GetValueDouble());

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

  //TODO: reimplement
  /*if (**this->provideFeedbackP)
  {
    this->feedback = new dJointFeedback;
    dJointSetFeedback(this->jointId, this->feedback);
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
// Get the body to which the joint is attached according the _index
BodyPtr ODEJoint::GetJointBody( int index ) const
{
  BodyPtr result;

  if ( index==0 || index==1 )
  {
    ODEBodyPtr odeBody1 = boost::shared_dynamic_cast<ODEBody>(this->childBody);
    ODEBodyPtr odeBody2 = boost::shared_dynamic_cast<ODEBody>(this->parentBody);
    if (odeBody1 != NULL &&
        dJointGetBody( this->jointId, index ) == odeBody1->GetODEId())
      result = this->childBody;
    else if (odeBody2)
      result = this->parentBody;
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Determines of the two bodies are connected by a joint
bool ODEJoint::AreConnected( BodyPtr one, BodyPtr two ) const
{
  ODEBodyPtr odeBody1 = boost::shared_dynamic_cast<ODEBody>(one);
  ODEBodyPtr odeBody2 = boost::shared_dynamic_cast<ODEBody>(two);

  if (odeBody1 == NULL || odeBody2 == NULL)
    gzthrow("ODEJoint requires ODE bodies\n");

  return dAreConnected( odeBody1->GetODEId(), odeBody2->GetODEId() );
}

//////////////////////////////////////////////////////////////////////////////
// The default function does nothing. This should be overriden in the
// child classes where appropriate
double ODEJoint::GetParam( int /*parameter*/ ) const
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Attach the two bodies with this joint
void ODEJoint::Attach(BodyPtr parent, BodyPtr child)
{
  Joint::Attach(parent, child);

  ODEBodyPtr odechild = boost::shared_dynamic_cast<ODEBody>(this->childBody);
  ODEBodyPtr odeparent = boost::shared_dynamic_cast<ODEBody>(this->parentBody);

  if (odechild == NULL && odeparent == NULL)
    gzthrow("ODEJoint requires at least one ODE body\n");

  
  if (!odechild && odeparent)
  {
    dJointAttach( this->jointId, 0, odeparent->GetODEId() );
  }
  else if (odechild && !odeparent)
  {
    dJointAttach( this->jointId, odechild->GetODEId(), 0 );
  }
  else if (odechild && odeparent)
  {
    dJointAttach( this->jointId, odechild->GetODEId(), odeparent->GetODEId() );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Detach this joint from all bodies
void ODEJoint::Detach()
{
  this->childBody.reset();
  this->parentBody.reset();
  dJointAttach( this->jointId, 0, 0 );
}

//////////////////////////////////////////////////////////////////////////////
// By default this does nothing. It should be overridden in child classes
// where appropriate
void ODEJoint::SetParam(int /*parameter*/, double /*value*/)
{
  if (this->childBody) 
    this->childBody->SetEnabled(true);
  if (this->parentBody) 
    this->parentBody->SetEnabled(true);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the ERP of this joint
void ODEJoint::SetERP(double newERP)
{
  this->SetParam(dParamSuspensionERP, newERP);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double ODEJoint::GetERP()
{
  return this->GetParam(dParamSuspensionERP);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the CFM of this joint
void ODEJoint::SetCFM(double newCFM)
{
  this->SetParam(dParamSuspensionCFM, newCFM);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double ODEJoint::GetCFM()
{
  return this->GetParam(dParamSuspensionCFM);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the feedback data structure for this joint, if set
dJointFeedback *ODEJoint::GetFeedback()
{
  return dJointGetFeedback(this->jointId);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the high stop of an axis(index)
void ODEJoint::SetHighStop(int index, math::Angle angle)
{
  switch (index)
  {
    case 0:
      this->SetParam(dParamHiStop, angle.GetAsRadian());
    case 1:
      this->SetParam(dParamHiStop2, angle.GetAsRadian());
    case 2:
      this->SetParam(dParamHiStop3, angle.GetAsRadian());
  };
}

////////////////////////////////////////////////////////////////////////////////
/// Set the low stop of an axis(index)
void ODEJoint::SetLowStop(int index, math::Angle angle)
{
  switch (index)
  {
    case 0:
      this->SetParam(dParamLoStop, angle.GetAsRadian());
    case 1:
      this->SetParam(dParamLoStop2, angle.GetAsRadian());
    case 2:
      this->SetParam(dParamLoStop3, angle.GetAsRadian());
  };
}

////////////////////////////////////////////////////////////////////////////////
/// Get the high stop of an axis(index).
math::Angle ODEJoint::GetHighStop(int index)
{
  switch (index)
  {
    case 0:
      return this->GetParam(dParamHiStop);
    case 1:
      return this->GetParam(dParamHiStop2);
    case 2:
      return this->GetParam(dParamHiStop3);
  };

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the low stop of an axis(index).
math::Angle ODEJoint::GetLowStop(int index)
{
  switch (index)
  {
    case 0:
      return this->GetParam(dParamLoStop);
    case 1:
      return this->GetParam(dParamLoStop2);
    case 2:
      return this->GetParam(dParamLoStop3);
  };

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force the joint applies to the first body
math::Vector3 ODEJoint::GetBodyForce(unsigned int index) const
{
  math::Vector3 result;
  dJointFeedback *feedback = dJointGetFeedback(this->jointId);

  if (index == 0)
    result.Set(feedback->f1[0], feedback->f1[1], feedback->f1[2]);
  else
    result.Set(feedback->f2[0], feedback->f2[1], feedback->f2[2]);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the torque the joint applies to the first body
math::Vector3 ODEJoint::GetBodyTorque(unsigned int index) const
{
  math::Vector3 result;
  dJointFeedback *feedback = dJointGetFeedback(this->jointId);

  if (index == 0)
    result.Set(feedback->t1[0], feedback->t1[1], feedback->t1[2]);
  else
    result.Set(feedback->t2[0], feedback->t2[1], feedback->t2[2]);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set a parameter for the joint
void ODEJoint::SetAttribute( Attribute _attr, int /*_index*/, double _value)
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
