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

  this->showJointsConnection = event::Events::ConnectShowJointsSignal(boost::bind(&Joint::ShowJoints, this, _1) );
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load a joint
void Joint::Load(sdf::ElementPtr &_sdf)
{
  Base::Load(_sdf);

  //this->node->Init(this->GetWorld()->GetName());
  //this->vis_pub = transport::advertise<msgs::Visual>("~/visual");

  std::ostringstream visname;

  std::string parentName = _sdf->GetElement("parent")->GetValueString("link");
  std::string childName = _sdf->GetElement("child")->GetValueString("link");

  if (this->model)
  {
    visname << this->model->GetScopedName() 
            << "::" << this->GetName() << "_VISUAL";

    this->childBody = this->model->GetBody( childName );
    this->parentBody = this->model->GetBody( parentName );
  }
  else
  {
    visname << this->GetName() << "_VISUAL";
    this->childBody = boost::shared_dynamic_cast<Body>(
        this->GetWorld()->GetByName( childName) );

    this->parentBody = boost::shared_dynamic_cast<Body>(
        this->GetWorld()->GetByName( parentName ));
  }

  this->anchorBody = this->parentBody;

  if (!this->parentBody && parentName != std::string("world"))
    gzthrow("Couldn't Find Parent Body[" + parentName );

  if (!this->childBody && childName != std::string("world"))
    gzthrow("Couldn't Find Child Body[" + childName);

  math::Pose offset = _sdf->GetElement("origin")->GetValuePose("pose");

  // setting anchor relative to gazebo body frame origin
  this->anchorPos = (offset + this->anchorBody->GetWorldPose()).pos ;
}

void Joint::Init()
{
  this->Attach(this->childBody, this->parentBody);

  //TODO: Instead of Creating a multiple visual message. Send
  //a JointMessage.
  /// Add a renderable for the joint
  /*msgs::Visual msg;
  msgs::Init(msg, visname.str());
  msg.set_parent_id( this->GetName() );
  msg.set_render_type( msgs::Visual::MESH_RESOURCE );
  msgs::Set(msg.mutable_pose()->mutable_position(), this->anchorPos );
  msgs::Set(msg.mutable_pose()->mutable_orientation(), common::Quatern(1,0,0,0) );
  msg.set_cast_shadows( false );
  msg.set_mesh( "joint_anchor" );
  msg.set_material( "Gazebo/JointAnchor" );
  msg.set_visible( false );
  this->vis_pub->Publish(msg);
  this->visual = msg.header().str_id();
 
  msgs::Init(msg, visname.str() + "/line1"); 
  msg.set_parent_id( visname.str() );
  msg.set_render_type( msgs::Visual::LINE_LIST );
  msg.set_material( "Gazebo/BlueGlow" );
  this->vis_pub->Publish(msg);
  this->line1 = msg.header().str_id();

  msgs::Init(msg, visname.str() + "/line2"); 
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
/// Update the joint
void Joint::Update()
{
  this->jointUpdateSignal();

  //TODO: Evaluate impact of this code on performance
  /*if (this->showJoints)
  {
    msgs::Visual msg;
    msgs::Init(msg, this->visual);
    msgs::Set(msg.mutable_pose()->mutable_position(), this->anchorPos);
    msgs::Set(msg.mutable_pose()->mutable_orientation(), common::Quatern(1,0,0,0) );
    this->vis_pub->Publish(msg);

    if (this->childBody) 
    {
      msgs::Init(msg, this->line1);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      msgs::Set(pt, this->childBody->GetWorldPose().pos - this->anchorPos );

      this->vis_pub->Publish(msg);
    }

    if (this->parentBody)
    {
      msgs::Init(msg, this->line2);
      msgs::Point *pt;

      pt = msg.add_points();
      pt->set_x(0);
      pt->set_y(0);
      pt->set_z(0);

      pt = msg.add_points();
      msgs::Set(pt, this->parentBody->GetWorldPose().pos - this->anchorPos);
      this->vis_pub->Publish(msg);
    }
  }*/
}


//////////////////////////////////////////////////////////////////////////////
// Set the joint to show visuals
void Joint::ShowJoints(const bool & /*s_*/)
{
  /*msgs::Visual msg;
  msgs::Init(msg, this->visual);
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
