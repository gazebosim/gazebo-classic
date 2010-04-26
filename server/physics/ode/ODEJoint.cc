/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: The ODE base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 12 Oct 2009
 * SVN: $Id$
 */

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

  double h = this->physics->GetStepTime().Double();
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
    default:
      gzerr(0) << "Unable to handle joint attribute[" << attr << "]\n";
      break;
  };
}
