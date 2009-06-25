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
/* Desc: The base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#include "PhysicsEngine.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreDynamicLines.hh"
#include "Global.hh"
#include "Body.hh"
#include "Model.hh"
#include "World.hh"
#include "Joint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
Joint::Joint()
  : Common()
{
  this->visual = NULL;
  this->model = NULL;

  Param::Begin(&this->parameters);
  this->erpP = new ParamT<double>("erp",0.4,0);
  this->cfmP = new ParamT<double>("cfm",10e-3,0);
  this->stopKpP = new ParamT<double>("stopKp",1000000.0,0);
  this->stopKdP = new ParamT<double>("stopKd",1.0,0);
  this->body1NameP = new ParamT<std::string>("body1",std::string(),1);
  this->body2NameP = new ParamT<std::string>("body2",std::string(),1);
  this->anchorBodyNameP = new ParamT<std::string>("anchor",std::string(),0);
  this->anchorOffsetP = new ParamT<Vector3>("anchorOffset",Vector3(0,0,0), 0);
  this->provideFeedbackP = new ParamT<bool>("provideFeedback", false, 0);
  this->fudgeFactorP = new ParamT<double>( "fudgeFactor", 1.0, 0 );
  Param::End();

  this->body1 = NULL;
  this->body2 = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
  dJointDestroy( this->jointId );
  delete this->erpP;
  delete this->cfmP;
  delete this->stopKpP;
  delete this->stopKdP;
  delete this->body1NameP;
  delete this->body2NameP;
  delete this->anchorBodyNameP;
  delete this->anchorOffsetP;
  delete this->provideFeedbackP;
  delete this->fudgeFactorP;
}


//////////////////////////////////////////////////////////////////////////////
// Get the type of the joint
Joint::Type Joint::GetType() const
{
  return this->type;
}

//////////////////////////////////////////////////////////////////////////////
// Load a joint
void Joint::Load(XMLConfigNode *node)
{
  // Name the joint
  this->nameP->Load(node);

  this->body1NameP->Load(node);
  this->body2NameP->Load(node);
  this->anchorBodyNameP->Load(node);
  this->anchorOffsetP->Load(node);
  this->erpP->Load(node);
  this->cfmP->Load(node);
  this->stopKpP->Load(node);
  this->stopKdP->Load(node);
  this->provideFeedbackP->Load(node);
  this->fudgeFactorP->Load(node);

  Body *body1 = this->model->GetBody( **(this->body1NameP));
  Body *body2 = this->model->GetBody(**(this->body2NameP));
  Body *anchorBody = this->model->GetBody(**(this->anchorBodyNameP));

  if (!body1)
    gzthrow("Couldn't Find Body[" + node->GetString("body1","",1));

  if (!body2)
    gzthrow("Couldn't Find Body[" + node->GetString("body2","",1));

  // setting anchor relative to gazebo body frame origin
  Vector3 anchorVec = anchorBody->GetPose().pos + **(this->anchorOffsetP);

  double h = World::Instance()->GetPhysicsEngine()->GetStepTime();
  double stopErp = h * (**this->stopKpP) / (h * (**this->stopKpP) + (**this->stopKdP));
  double stopCfm = 1.0 / (h * (**this->stopKpP) + (**this->stopKdP));
  this->SetParam(dParamStopERP, stopErp);
  this->SetParam(dParamStopCFM, stopCfm);

  // Set joint parameters
  this->SetParam(dParamSuspensionERP, **(this->erpP));
  this->SetParam(dParamCFM, **(this->cfmP));
  this->SetParam(dParamFudgeFactor, **(this->fudgeFactorP));
  this->SetParam(dParamVel,0);
  this->SetParam(dParamFMax,0);
  this->SetParam(dParamBounce, 0);

  this->Attach(body1,body2);

  std::ostringstream visname;
  visname << this->model->GetScopedName() << "::" << this->GetName() << "_VISUAL";

  /// Add a renderable for the joint
  this->visual = OgreCreator::Instance()->CreateVisual(
      visname.str(), NULL);
  if (this->visual)
  {
    this->visual->SetCastShadows(false);
    this->visual->AttachMesh("joint_anchor");
    this->visual->SetMaterial("Gazebo/JointAnchor");
    this->visual->SetVisible(false);

    this->line1 = OgreCreator::Instance()->CreateDynamicLine(OgreDynamicRenderable::OT_LINE_LIST);
    this->line2 = OgreCreator::Instance()->CreateDynamicLine(OgreDynamicRenderable::OT_LINE_LIST);

    this->line1->setMaterial("Gazebo/BlueEmissive");
    this->line2->setMaterial("Gazebo/BlueEmissive");

    this->visual->AttachObject(this->line1);
    this->visual->AttachObject(this->line2);

    this->line1->AddPoint(Vector3(0,0,0));
    this->line1->AddPoint(Vector3(0,0,0));
    this->line2->AddPoint(Vector3(0,0,0));
    this->line2->AddPoint(Vector3(0,0,0));
  }

  if (**this->provideFeedbackP)
  {
    this->feedback = new dJointFeedback;
    dJointSetFeedback(this->jointId, this->feedback);
  }

  this->LoadChild(node);

  // Set the anchor vector
  if (anchorBody)
  {
    this->SetAnchor(anchorVec);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void Joint::Save(std::string &prefix, std::ostream &stream)
{
  std::string typeName;

  switch (this->type)
  {
    case SLIDER: typeName="slider"; break;
    case HINGE: typeName = "hinge"; break;
    case HINGE2: typeName = "hinge2"; break;
    case BALL: typeName = "ball"; break;
    case UNIVERSAL: typeName = "universal"; break;
  }

  stream << prefix << "<joint:" << typeName << " name=\"" << **(this->nameP) << "\">\n";
  stream << prefix << "  " << *(this->body1NameP) << "\n";
  stream << prefix << "  " << *(this->body2NameP) << "\n";
  stream << prefix << "  " << *(this->anchorBodyNameP) << "\n";
  stream << prefix << "  " << *(this->anchorOffsetP) << "\n";

  stream << prefix << "  " << *(this->erpP) << "\n";
  stream << prefix << "  " << *(this->cfmP) << "\n";
  stream << prefix << "  " << *(this->fudgeFactorP) << "\n";

  std::string p = prefix + "  ";
  this->SaveChild(p,stream);

  stream << prefix << "</joint:" << typeName << ">\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void Joint::Update()
{
//TODO: Evaluate impact of this code on performance
  if (this->visual)
    this->visual->SetVisible(World::Instance()->GetShowJoints());

  if (!World::Instance()->GetShowJoints())
    return;

  if (this->visual)
    this->visual->SetPosition(this->GetAnchor());

  Vector3 start;
  if (this->body1)
  {
    start = this->body1->GetPose().pos - this->GetAnchor();

    if (this->line1)
      this->line1->SetPoint(0, start);
  }

  if (this->body2)
  {
    start = this->body2->GetPose().pos - this->GetAnchor();
    if (this->line2)
      this->line2->SetPoint(0, start);
  }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Reset the joint
void Joint::Reset()
{
  this->ResetChild();
}


//////////////////////////////////////////////////////////////////////////////
// Set the model this joint belongs too
void Joint::SetModel(Model *model)
{
  this->model = model;
}

//////////////////////////////////////////////////////////////////////////////
// Get the body to which the joint is attached according the _index
Body *Joint::GetJointBody( int index ) const
{
  Body *result=0;

  if ( index==0 || index==1 )
  {
    if (this->body1 &&
        dJointGetBody( this->jointId, index ) == this->body1->GetId())
      result = this->body1;
    else if (this->body2)
      result = this->body2;
  }

  return result;
}


//////////////////////////////////////////////////////////////////////////////
// Determines of the two bodies are connected by a joint
bool Joint::AreConnected( Body *one, Body *two ) const
{
  return dAreConnected( one->GetId(), two->GetId() );
}


//////////////////////////////////////////////////////////////////////////////
// The default function does nothing. This should be overriden in the
// child classes where appropriate
double Joint::GetParam( int /*parameter*/ ) const
{
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Make this joint a fixed joint, use this only when absolutely necessary
void Joint::SetFixed()
{
  dJointSetFixed( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Attach the two bodies with this joint
void Joint::Attach( Body *one, Body *two )
{
  if (!one && two)
  {
    dJointAttach( this->jointId, 0, two->GetId() );
    this->body2 = two;
  }
  else if (one && !two)
  {
    dJointAttach( this->jointId, one->GetId(), 0 );
    this->body1 = one;
  }
  else if (one && two)
  {
    dJointAttach( this->jointId, one->GetId(), two->GetId() );
    this->body1 = one;
    this->body2 = two;
  }
}


//////////////////////////////////////////////////////////////////////////////
// Detach this joint from all bodies
void Joint::Detach()
{
  this->body1 = NULL;
  this->body2 = NULL;
  dJointAttach( this->jointId, 0, 0 );
  return;
}


//////////////////////////////////////////////////////////////////////////////
// By default this does nothing. It should be overridden in child classes
// where appropriate
void Joint::SetParam(int /*parameter*/, double /*value*/)
{
}

//////////////////////////////////////////////////////////////////////////////
/// Get the name of this joint
std::string Joint::GetName() const
{
  return this->nameP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of this joint
void Joint::SetName(const std::string &newName)
{
  this->nameP->SetValue(newName);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the ERP of this joint
void Joint::SetERP(double newERP)
{
  this->SetParam(dParamSuspensionERP, newERP);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double Joint::GetERP()
{
  return this->GetParam(dParamSuspensionERP);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the CFM of this joint
void Joint::SetCFM(double newCFM)
{
  this->SetParam(dParamSuspensionCFM, newCFM);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ERP of this joint
double Joint::GetCFM()
{
  return this->GetParam(dParamSuspensionCFM);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the feedback data structure for this joint, if set
dJointFeedback *Joint::GetFeedback()
{
  return dJointGetFeedback(this->jointId);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the high stop of an axis(index).
double Joint::GetHighStop(int index)
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
double Joint::GetLowStop(int index)
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
