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


  if (!this->parentBody && parentName != std::string("world"))
    gzthrow("Couldn't Find Parent Body[" + parentName );

  if (!this->childBody && childName != std::string("world"))
    gzthrow("Couldn't Find Child Body[" + childName);

  math::Pose offset;
  if (_sdf->HasElement("origin"))
    offset = _sdf->GetElement("origin")->GetValuePose("pose");

  // setting anchor relative to gazebo body frame origin
  if (this->childBody)
    this->anchorPos = (offset + this->childBody->GetWorldPose()).pos ;
  else
    this->anchorPos = math::Vector3(0,0,0); // default for world
}

void Joint::Init()
{
  this->Attach(this->parentBody, this->childBody);
  //this->Attach(this->childBody, this->parentBody);

  // Set the anchor vector
  this->SetAnchor(0, this->anchorPos);

  // Set joint axis
  if (this->sdf->HasElement("axis"))
    this->SetAxis(0, this->sdf->GetElement("axis")->GetValueVector3("xyz"));
  if (this->sdf->HasElement("axis2"))
    this->SetAxis(1, this->sdf->GetElement("axis2")->GetValueVector3("xyz"));

}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void Joint::Update()
{
  this->jointUpdateSignal();
}

//////////////////////////////////////////////////////////////////////////////
// Set the joint to show visuals
void Joint::ShowJoints(const bool & /*s_kk*/)
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

//////////////////////////////////////////////////////////////////////////////
/// Get the child link
BodyPtr Joint::GetChild() const
{
  return this->childBody;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the child link
BodyPtr Joint::GetParent() const
{
  return this->parentBody;
}
