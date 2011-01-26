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

#include "World.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Body.hh"
#include "ODEBody.hh"
#include "PhysicsEngine.hh"
#include "ODEJoint.hh"

using namespace gazebo;

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
void ODEJoint::Load(XMLConfigNode *node)
{
  Joint::Load(node);

  double h = this->GetWorld()->GetPhysicsEngine()->GetStepTime().Double();
  double stopErp = h * (**this->stopKpP) / (h * (**this->stopKpP) + (**this->stopKdP));
  double stopCfm = 1.0 / (h * (**this->stopKpP) + (**this->stopKdP));

  // Set joint parameters
  this->SetParam(dParamSuspensionERP, **(this->erpP));
  this->SetParam(dParamCFM, **(this->cfmP));
  this->SetParam(dParamFudgeFactor, **(this->fudgeFactorP));
  this->SetParam(dParamVel,0);
  this->SetParam(dParamFMax,0);
  this->SetParam(dParamBounce, 0);
  this->SetParam(dParamStopERP, stopErp);
  this->SetParam(dParamStopCFM, stopCfm);

  if (**this->provideFeedbackP)
  {
    this->feedback = new dJointFeedback;
    dJointSetFeedback(this->jointId, this->feedback);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the body to which the joint is attached according the _index
Body *ODEJoint::GetJointBody( int index ) const
{
  Body *result=0;

  if ( index==0 || index==1 )
  {
    ODEBody *odeBody1 = dynamic_cast<ODEBody*>(this->body1);
    ODEBody *odeBody2 = dynamic_cast<ODEBody*>(this->body2);
    if (odeBody1 != NULL &&
        dJointGetBody( this->jointId, index ) == odeBody1->GetODEId())
      result = this->body1;
    else if (odeBody2)
      result = this->body2;
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////
// Determines of the two bodies are connected by a joint
bool ODEJoint::AreConnected( Body *one, Body *two ) const
{
  ODEBody *odeBody1 = dynamic_cast<ODEBody*>(one);
  ODEBody *odeBody2 = dynamic_cast<ODEBody*>(two);

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
void ODEJoint::Attach(Body *one, Body *two)
{
  Joint::Attach(one, two);

  ODEBody *odeBody1 = dynamic_cast<ODEBody*>(this->body1);
  ODEBody *odeBody2 = dynamic_cast<ODEBody*>(this->body2);

  if (odeBody1 == NULL && odeBody2 == NULL)
    gzthrow("ODEJoint requires at least one ODE body\n");

  if (!odeBody1 && odeBody2)
  {
    dJointAttach( this->jointId, 0, odeBody2->GetODEId() );
  }
  else if (odeBody1 && !odeBody2)
  {
    dJointAttach( this->jointId, odeBody1->GetODEId(), 0 );
  }
  else if (odeBody1 && odeBody2)
  {
    dJointAttach( this->jointId, odeBody1->GetODEId(), odeBody2->GetODEId() );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Detach this joint from all bodies
void ODEJoint::Detach()
{
  this->body1 = NULL;
  this->body2 = NULL;
  dJointAttach( this->jointId, 0, 0 );
  return;
}

//////////////////////////////////////////////////////////////////////////////
// By default this does nothing. It should be overridden in child classes
// where appropriate
void ODEJoint::SetParam(int /*parameter*/, double /*value*/)
{
  if (this->body1) this->body1->SetEnabled(true);
  if (this->body2) this->body2->SetEnabled(true);
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
void ODEJoint::SetHighStop(int index, Angle angle)
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
void ODEJoint::SetLowStop(int index, Angle angle)
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
Angle ODEJoint::GetHighStop(int index)
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
Angle ODEJoint::GetLowStop(int index)
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
Vector3 ODEJoint::GetBodyForce(unsigned int index) const
{
  Vector3 result;
  dJointFeedback *feedback = dJointGetFeedback(this->jointId);

  if (index == 0)
    result.Set(feedback->f1[0], feedback->f1[1], feedback->f1[2]);
  else
    result.Set(feedback->f2[0], feedback->f2[1], feedback->f2[2]);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the torque the joint applies to the first body
Vector3 ODEJoint::GetBodyTorque(unsigned int index) const
{
  Vector3 result;
  dJointFeedback *feedback = dJointGetFeedback(this->jointId);

  if (index == 0)
    result.Set(feedback->t1[0], feedback->t1[1], feedback->t1[2]);
  else
    result.Set(feedback->t2[0], feedback->t2[1], feedback->t2[2]);

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set a parameter for the joint
void ODEJoint::SetAttribute( Attribute attr, int index, double value)
{
  switch (attr)
  {
    case FUDGE_FACTOR:
      this->SetParam(dParamFudgeFactor, value);
      break;
    case SUSPENSION_ERP:
      this->SetParam(dParamSuspensionERP, value);
      break;
    case SUSPENSION_CFM:
      this->SetParam(dParamSuspensionCFM, value);
      break;
    case STOP_ERP:
      this->SetParam(dParamStopERP, value);
      break;
    case STOP_CFM:
      this->SetParam(dParamStopCFM, value);
      break;
    case ERP:
      this->SetParam(dParamERP, value);
      break;
    case CFM:
      this->SetParam(dParamCFM, value);
      break;
    case FMAX:
      this->SetParam(dParamFMax, value);
      break;
    case VEL:
      this->SetParam(dParamVel, value);
      break;
    case HI_STOP:
      this->SetParam(dParamHiStop, value);
      break;
    case LO_STOP:
      this->SetParam(dParamLoStop, value);
      break;
    default:
      gzerr(0) << "Unable to handle joint attribute[" << attr << "]\n";
      break;
  };
}
