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
/* Desc: The base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#include "Simulator.hh"
#include "RenderTypes.hh"
#include "Events.hh"
#include "PhysicsEngine.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Global.hh"
#include "Body.hh"
#include "Model.hh"
#include "World.hh"
#include "Joint.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
Joint::Joint()
  : Common(NULL)
{
  this->vis_pub = Simulator::Instance()->Advertise<msgs::Visual>("/gazebo/visual");
  this->AddType(JOINT);
  this->model = NULL;
  this->showJoints = false;

  Param::Begin(&this->parameters);
  this->erpP = new ParamT<double>("erp",0.4,0);
  this->cfmP = new ParamT<double>("cfm",10e-3,0);
  this->stopKpP = new ParamT<double>("stop_kp",1000000.0,0);
  this->stopKdP = new ParamT<double>("stop_kd",1.0,0);
  this->body1NameP = new ParamT<std::string>("body1",std::string(),1);
  this->body2NameP = new ParamT<std::string>("body2",std::string(),1);
  this->anchorBodyNameP = new ParamT<std::string>("anchor",std::string(),0);
  this->anchorOffsetP = new ParamT<Vector3>("anchor_offset",Vector3(0,0,0), 0);
  this->provideFeedbackP = new ParamT<bool>("provide_feedback", false, 0);
  this->fudgeFactorP = new ParamT<double>( "fudge_factor", 1.0, 0 );
  Param::End();

  this->body1 = NULL;
  this->body2 = NULL;

  this->showJointsConnection = Events::ConnectShowJointsSignal(boost::bind(&Joint::ShowJoints, this, _1) );
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
  if (!this->visual.empty())
  {
    msgs::Visual msg;
    Message::Init(msg, this->visual);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }

  if (!this->line1.empty())
  {
    msgs::Visual msg;
    Message::Init(msg, this->line1);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }

  if (!this->line2.empty())
  {
    msgs::Visual msg;
    Message::Init(msg, this->line2);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }

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

  std::ostringstream visname;

  if (this->model)
  {
    visname << this->model->GetScopedName() << "::" << this->GetName() << "_VISUAL";

    this->body1 = this->model->GetBody( **(this->body1NameP));
    this->body2 = this->model->GetBody(**(this->body2NameP));
    this->anchorBody = this->model->GetBody(**(this->anchorBodyNameP));
  }
  else
  {
    visname << this->GetName() << "_VISUAL";
    this->body1 = dynamic_cast<Body*>(this->GetWorld()->GetByName( **(this->body1NameP) ));
    this->body2 = dynamic_cast<Body*>(this->GetWorld()->GetByName( **(this->body2NameP) ));
    this->anchorBody = dynamic_cast<Body*>(this->GetWorld()->GetByName( **(this->anchorBodyNameP) ));
  }

  if (!this->body1 && this->body1NameP->GetValue() != std::string("world"))
    gzthrow("Couldn't Find Body[" + node->GetString("body1","",1));

  if (!this->body2 && this->body2NameP->GetValue() != std::string("world"))
    gzthrow("Couldn't Find Body[" + node->GetString("body2","",1));

  // setting anchor relative to gazebo body frame origin
  this->anchorPos = (Pose3d(**(this->anchorOffsetP),Quatern()) + this->anchorBody->GetWorldPose()).pos ;

  this->Attach(this->body1, this->body2);

  /// Add a renderable for the joint
  msgs::Visual msg;
  Message::Init(msg, visname.str());
  msg.set_parent_id( this->GetName() );
  msg.set_render_type( msgs::Visual::MESH_RESOURCE );
  Message::Set(msg.mutable_pose()->mutable_position(), this->anchorPos );
  msg.set_cast_shadows( false );
  msg.set_mesh( "joint_anchor" );
  msg.set_material( "Gazebo/JointAnchor" );
  msg.set_visible( false );
  this->vis_pub->Publish(msg);
  this->visual = msg.header().str_id();
 
  Message::Init(msg, visname.str() + "/line1"); 
  msg.set_parent_id( visname.str() );
  msg.set_render_type( msgs::Visual::LINE_LIST );
  msg.set_material( "Gazebo/BlueGlow" );
  this->vis_pub->Publish(msg);
  this->line1 = msg.header().str_id();

  Message::Init(msg, visname.str() + "/line2"); 
  msg.set_parent_id( visname.str() );
  msg.set_render_type( msgs::Visual::LINE_LIST );
  msg.set_material( "Gazebo/BlueGlow" );
  this->vis_pub->Publish(msg);
  this->line2 = msg.header().str_id();


  // Set the anchor vector
  if (this->anchorBody)
  {
    this->SetAnchor(0, this->anchorPos);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void Joint::Save(std::string &prefix, std::ostream &stream)
{
  std::string typeName = EntityTypename[ (int)this->GetLeafType() ];

  stream << prefix << "<joint:" << typeName << " name=\"" << **(this->nameP) << "\">\n";
  stream << prefix << "  " << *(this->body1NameP) << "\n";
  stream << prefix << "  " << *(this->body2NameP) << "\n";
  stream << prefix << "  " << *(this->anchorBodyNameP) << "\n";
  stream << prefix << "  " << *(this->anchorOffsetP) << "\n";

  stream << prefix << "  " << *(this->erpP) << "\n";
  stream << prefix << "  " << *(this->cfmP) << "\n";
  stream << prefix << "  " << *(this->fudgeFactorP) << "\n";

  this->SaveJoint(prefix, stream);

  std::string p = prefix + "  ";

  stream << prefix << "</joint:" << typeName << ">\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void Joint::Update()
{
  this->jointUpdateSignal();

  //TODO: Evaluate impact of this code on performance
  if (this->showJoints)
  {
    msgs::Visual msg;
    Message::Init(msg, this->visual);
    Message::Set(msg.mutable_pose()->mutable_position(), this->anchorPos);
    this->vis_pub->Publish(msg);

    if (this->body1) 
    {
      Message::Init(msg, this->line1);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      Message::Set(pt, this->body1->GetWorldPose().pos - this->anchorPos );

      this->vis_pub->Publish(msg);
    }

    if (this->body2)
    {
      Message::Init(msg, this->line2);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      Message::Set(pt, this->body2->GetWorldPose().pos - this->anchorPos);
      this->vis_pub->Publish(msg);
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
// Set the joint to show visuals
void Joint::ShowJoints(const bool &s)
{
  msgs::Visual msg;
  Message::Init(msg, this->visual);
  msg.set_visible(s);
  this->vis_pub->Publish(msg);
  this->showJoints = s;
}

//////////////////////////////////////////////////////////////////////////////
/// Reset the joint
void Joint::Reset()
{
  this->SetForce(0,0);
  this->SetMaxForce(0,0);
  this->SetVelocity(0,0);
}

//////////////////////////////////////////////////////////////////////////////
/// Attach the two bodies with this joint
void Joint::Attach( Body *one, Body *two )
{
  this->body1 = one;
  this->body2 = two;
}


//////////////////////////////////////////////////////////////////////////////
// Set the model this joint belongs too
void Joint::SetModel(Model *model)
{
  this->model = model;
  this->SetWorld(this->model->GetWorld());
}
