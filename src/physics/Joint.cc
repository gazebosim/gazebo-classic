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
 */

#include "transport/Transport.hh"
#include "transport/Publisher.hh"

#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/XMLConfig.hh"

#include "physics/PhysicsEngine.hh"
#include "physics/Body.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
Joint::Joint()
  : Base(BasePtr())
{
  this->AddType(Base::JOINT);
  this->showJoints = false;

  common::Param::Begin(&this->parameters);
  this->erpP = new common::ParamT<double>("erp",0.4,0);
  this->cfmP = new common::ParamT<double>("cfm",10e-3,0);
  this->stopKpP = new common::ParamT<double>("stop_kp",1000000.0,0);
  this->stopKdP = new common::ParamT<double>("stop_kd",1.0,0);
  this->parentNameP = new common::ParamT<std::string>("link",std::string(),1);
  this->childNameP = new common::ParamT<std::string>("link",std::string(),1);
  this->anchorOffsetP = new common::ParamT<common::Vector3>("anchor_offset",common::Vector3(0,0,0), 0);
  this->provideFeedbackP = new common::ParamT<bool>("provide_feedback", false, 0);
  this->fudgeFactorP = new common::ParamT<double>( "fudge_factor", 1.0, 0 );
  common::Param::End();

  this->showJointsConnection = event::Events::ConnectShowJointsSignal(boost::bind(&Joint::ShowJoints, this, _1) );
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
  /*if (!this->visual.empty())
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->visual);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }

  if (!this->line1.empty())
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->line1);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }

  if (!this->line2.empty())
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->line2);
    msg.set_action( msgs::Visual::DELETE );
    this->vis_pub->Publish(msg);
  }
  */

  delete this->erpP;
  delete this->cfmP;
  delete this->stopKpP;
  delete this->stopKdP;
  delete this->parentNameP;
  delete this->childNameP;
  delete this->anchorOffsetP;
  delete this->provideFeedbackP;
  delete this->fudgeFactorP;
}

//////////////////////////////////////////////////////////////////////////////
// Load a joint
void Joint::Load(common::XMLConfigNode *node)
{
  Base::Load(node);

  //this->node->Init(this->GetWorld()->GetName());
  //this->vis_pub = transport::advertise<msgs::Visual>("~/visual");

  this->childNameP->Load(node->GetChild("child"));
  this->parentNameP->Load(node->GetChild("parent"));
  this->anchorOffsetP->Load(node);
  this->erpP->Load(node);
  this->cfmP->Load(node);
  this->stopKpP->Load(node);
  this->stopKdP->Load(node);
  this->provideFeedbackP->Load(node);
  this->fudgeFactorP->Load(node);

  std::ostringstream visname;

  gzdbg << "Parent[" << **this->parentNameP << "] Child[" << **this->childNameP << "]\n";

  if (this->model)
  {
    visname << this->model->GetScopedName() << "::" << this->GetName() << "_VISUAL";

    this->childBody = this->model->GetBody( **(this->childNameP) );
    this->parentBody = this->model->GetBody( **(this->parentNameP) );
  }
  else
  {
    visname << this->GetName() << "_VISUAL";
    this->childBody = boost::shared_dynamic_cast<Body>(
        this->GetWorld()->GetByName( **(this->childNameP) ));

    this->parentBody = boost::shared_dynamic_cast<Body>(
        this->GetWorld()->GetByName( **(this->parentNameP) ));
  }

  this->anchorBody = this->parentBody;

  if (!this->parentBody && **this->parentNameP != std::string("world"))
    gzthrow("Couldn't Find Parent Body[" + **this->parentNameP );

  if (!this->childBody && **this->childNameP != std::string("world"))
    gzthrow("Couldn't Find Child Body[" + **this->childNameP);

  // setting anchor relative to gazebo body frame origin
  this->anchorPos = (common::Pose3d(**(this->anchorOffsetP),common::Quatern()) + this->anchorBody->GetWorldPose()).pos ;

}

void Joint::Init()
{
  this->Attach(this->childBody, this->parentBody);

  //TODO: Instead of Creating a multiple visual message. Send
  //a JointMessage.
  /// Add a renderable for the joint
  /*msgs::Visual msg;
  common::Message::Init(msg, visname.str());
  msg.set_parent_id( this->GetName() );
  msg.set_render_type( msgs::Visual::MESH_RESOURCE );
  common::Message::Set(msg.mutable_pose()->mutable_position(), this->anchorPos );
  common::Message::Set(msg.mutable_pose()->mutable_orientation(), common::Quatern(1,0,0,0) );
  msg.set_cast_shadows( false );
  msg.set_mesh( "joint_anchor" );
  msg.set_material( "Gazebo/JointAnchor" );
  msg.set_visible( false );
  this->vis_pub->Publish(msg);
  this->visual = msg.header().str_id();
 
  common::Message::Init(msg, visname.str() + "/line1"); 
  msg.set_parent_id( visname.str() );
  msg.set_render_type( msgs::Visual::LINE_LIST );
  msg.set_material( "Gazebo/BlueGlow" );
  this->vis_pub->Publish(msg);
  this->line1 = msg.header().str_id();

  common::Message::Init(msg, visname.str() + "/line2"); 
  msg.set_parent_id( visname.str() );
  msg.set_render_type( msgs::Visual::LINE_LIST );
  msg.set_material( "Gazebo/BlueGlow" );
  this->vis_pub->Publish(msg);
  this->line2 = msg.header().str_id();
  */


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
  std::string typeName;// =Base::EntityTypename[ (int)this->GetLeafType() ];

  stream << prefix << "<joint:" << typeName << " name=\"" << this->GetName() << "\">\n";
  stream << prefix << "  " << *(this->childNameP) << "\n";
  stream << prefix << "  " << *(this->parentNameP) << "\n";
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
  /*if (this->showJoints)
  {
    msgs::Visual msg;
    common::Message::Init(msg, this->visual);
    common::Message::Set(msg.mutable_pose()->mutable_position(), this->anchorPos);
    common::Message::Set(msg.mutable_pose()->mutable_orientation(), common::Quatern(1,0,0,0) );
    this->vis_pub->Publish(msg);

    if (this->childBody) 
    {
      common::Message::Init(msg, this->line1);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      common::Message::Set(pt, this->childBody->GetWorldPose().pos - this->anchorPos );

      this->vis_pub->Publish(msg);
    }

    if (this->parentBody)
    {
      common::Message::Init(msg, this->line2);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      common::Message::Set(pt, this->parentBody->GetWorldPose().pos - this->anchorPos);
      this->vis_pub->Publish(msg);
    }
  }*/
}


//////////////////////////////////////////////////////////////////////////////
// Set the joint to show visuals
void Joint::ShowJoints(const bool &s)
{
  /*msgs::Visual msg;
  common::Message::Init(msg, this->visual);
  msg.set_visible(s);
  this->vis_pub->Publish(msg);
  this->showJoints = s;
  */
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
void Joint::Attach( BodyPtr parent, BodyPtr child )
{
  this->parentBody = parent;
  this->childBody = child;
}


//////////////////////////////////////////////////////////////////////////////
// Set the model this joint belongs too
void Joint::SetModel(ModelPtr model)
{
  this->model = model;
  this->SetWorld(this->model->GetWorld());
}
