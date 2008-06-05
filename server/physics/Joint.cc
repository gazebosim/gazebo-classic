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
#include "OgreVisual.hh"
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
{
  this->visual = NULL;
  this->model = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
  dJointDestroy( this->jointId );
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
  this->SetName(node->GetString("name","",1));

  this->LoadChild(node);

  // Set joint parameters
  this->SetParam(dParamSuspensionERP, node->GetDouble("erp",0.4,0));
  this->SetParam(dParamSuspensionCFM, node->GetDouble("cfm",0.8,0));

  /// Add a renderable for the joint
  this->visual = new OgreVisual(this->model->GetVisualNode());
  this->visual->AttachMesh("joint_anchor");
  this->visual->SetVisible(false);


  this->line1 = new OgreDynamicLines(OgreDynamicRenderable::OT_LINE_LIST);
  this->line2 = new OgreDynamicLines(OgreDynamicRenderable::OT_LINE_LIST);
  this->line1->setMaterial("Gazebo/BlueEmissive");
  this->line2->setMaterial("Gazebo/BlueEmissive");

  this->visual->AttachObject(this->line1);
  this->visual->AttachObject(this->line2);

  this->line1->AddPoint(Vector3(0,0,0));
  this->line1->AddPoint(Vector3(0,0,0));
  this->line2->AddPoint(Vector3(0,0,0));
  this->line2->AddPoint(Vector3(0,0,0));

}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void Joint::Update()
{
//TODO: Evaluate impact of this code on performance
  this->visual->SetVisible(World::Instance()->GetShowJoints());

  if (!World::Instance()->GetShowJoints())
    return;

  this->visual->SetPosition(this->GetAnchor());

  Vector3 start;
  if (this->body1)
  {
    start = this->body1->GetPose().pos - this->GetAnchor();
    this->line1->SetPoint(0, start);
    this->line1->Update();
  }

  if (this->body2)
  {
    start = this->body2->GetPose().pos - this->GetAnchor();
    this->line2->SetPoint(0, start);
    this->line2->Update();
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
    if (dJointGetBody( this->jointId, index ) == this->body1->GetId())
      result = this->body1;
    else
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
  return this->name;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of this joint
void Joint::SetName(const std::string &newName)
{
  this->name = newName;
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
