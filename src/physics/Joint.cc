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
#include "physics/Link.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Joint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
Joint::Joint()
  : Base(BasePtr())
{
  this->AddType(Base::JOINT);
  this->showJoints = false;

  this->showJointsConnection =
    event::Events::ConnectShowJoints(boost::bind(&Joint::ShowJoints, this, _1));
}


//////////////////////////////////////////////////
Joint::~Joint()
{
}

//////////////////////////////////////////////////
void Joint::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::ostringstream visname;

  std::string parentName = _sdf->GetElement("parent")->GetValueString("link");
  std::string childName = _sdf->GetElement("child")->GetValueString("link");

  if (this->model)
  {
    visname << this->model->GetScopedName()
            << "::" << this->GetName() << "_VISUAL";

    this->childLink = this->model->GetLink(childName);
    this->parentLink = this->model->GetLink(parentName);
  }
  else
  {
    visname << this->GetName() << "_VISUAL";
    this->childLink = boost::shared_dynamic_cast<Link>(
        this->GetWorld()->GetByName(childName));

    this->parentLink = boost::shared_dynamic_cast<Link>(
        this->GetWorld()->GetByName(parentName));
  }

  BasePtr myBase = shared_from_this();

  if (!this->parentLink)
  {
    if (parentName != std::string("world"))
    {
      gzthrow("Couldn't Find Parent Link[" + parentName);
    }
  }
  else
  {
    this->parentLink->AddChildJoint(boost::shared_static_cast<Joint>(myBase));
  }

  if (!this->childLink && childName != std::string("world"))
    gzthrow("Couldn't Find Child Link[" + childName);

  this->childLink->AddParentJoint(boost::shared_static_cast<Joint>(myBase));

  math::Pose offset;
  if (_sdf->HasElement("origin"))
    offset = _sdf->GetElement("origin")->GetValuePose("pose");

  // setting anchor relative to gazebo link frame origin
  if (this->childLink)
    this->anchorPos = (offset + this->childLink->GetWorldPose()).pos;
  else
    this->anchorPos = math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void Joint::Init()
{
  this->Attach(this->parentLink, this->childLink);

  // Set the anchor vector
  this->SetAnchor(0, this->anchorPos);

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    this->SetAxis(0, axisElem->GetValueVector3("xyz"));
    //if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(0, limitElem->GetValueDouble("upper"));
      this->SetLowStop(0, limitElem->GetValueDouble("lower"));
      this->SetHighStop(0, limitElem->GetValueDouble("upper"));
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    this->SetAxis(1, axisElem->GetValueVector3("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters  are set properly.
      // This is taken from the ODE wiki.
      limitElem = axisElem->GetElement("limit");
      this->SetHighStop(1, limitElem->GetValueDouble("upper"));
      this->SetLowStop(1, limitElem->GetValueDouble("lower"));
      this->SetHighStop(1, limitElem->GetValueDouble("upper"));
    }
  }

  if (this->parentLink)
  {
    math::Pose modelPose = this->parentLink->GetModel()->GetWorldPose();

    // Set joint axis
    if (this->sdf->HasElement("axis"))
    {
      this->SetAxis(0, modelPose.rot.RotateVector(
            this->sdf->GetElement("axis")->GetValueVector3("xyz")));
    }

    if (this->sdf->HasElement("axis2"))
    {
      this->SetAxis(1, modelPose.rot.RotateVector(
            this->sdf->GetElement("axis2")->GetValueVector3("xyz")));
    }
  }
  else
  {
    if (this->sdf->HasElement("axis"))
    {
      this->SetAxis(0, this->sdf->GetElement("axis")->GetValueVector3("xyz"));
      gzwarn << "joint [" << this->GetName() << "] has a non-real parentLink ["
             << sdf->GetElement("parent")->GetValueString("link")
             << "], loading axis from SDF ["
             << this->sdf->GetElement("axis")->GetValueVector3("xyz")
             << "]\n";
    }
    if (this->sdf->HasElement("axis2"))
    {
      this->SetAxis(1, this->sdf->GetElement("axis2")->GetValueVector3("xyz"));
      gzwarn << "joint [" << this->GetName() << "] has a non-real parentLink ["
             << sdf->GetElement("parent")->GetValueString("link")
             << "], loading axis2 from SDF ["
             << this->sdf->GetElement("axis2")->GetValueVector3("xyz")
             << "]\n";
    }
  }
}

//////////////////////////////////////////////////
math::Vector3 Joint::GetLocalAxis(int _index) const
{
  math::Vector3 vec;

  if (_index == 0 && this->sdf->HasElement("axis"))
    vec = this->sdf->GetElement("axis")->GetValueVector3("xyz");
  else if (this->sdf->HasElement("axis2"))
    vec = this->sdf->GetElement("axis2")->GetValueVector3("xyz");

  return vec;
}

//////////////////////////////////////////////////
void Joint::Update()
{
  this->jointUpdate();
}

//////////////////////////////////////////////////
void Joint::UpdateParameters(sdf::ElementPtr _sdf)
{
  Base::UpdateParameters(_sdf);
}

//////////////////////////////////////////////////
void Joint::ShowJoints(const bool & /*s_kk*/)
{
  /*msgs::Visual msg;
  msg.set_name(this->visual);
  msg.set_visible(s);
  this->vis_pub->Publish(msg);
  this->showJoints = s;
  */
}

//////////////////////////////////////////////////
void Joint::Reset()
{
  this->SetMaxForce(0, 0);
  this->SetVelocity(0, 0);
  this->Init();
  this->staticAngle.SetFromRadian(0);
}

//////////////////////////////////////////////////
void Joint::Attach(LinkPtr _parent, LinkPtr _child)
{
  this->parentLink = _parent;
  this->childLink = _child;
}


//////////////////////////////////////////////////
void Joint::SetModel(ModelPtr _model)
{
  this->model = _model;
  this->SetWorld(this->model->GetWorld());
}

//////////////////////////////////////////////////
LinkPtr Joint::GetChild() const
{
  return this->childLink;
}

//////////////////////////////////////////////////
LinkPtr Joint::GetParent() const
{
  return this->parentLink;
}

//////////////////////////////////////////////////
void Joint::FillJointMsg(msgs::Joint &_msg)
{
  _msg.set_name(this->GetScopedName());

  if (this->sdf->HasElement("origin"))
  {
    msgs::Set(_msg.mutable_pose(),
              this->sdf->GetElement("origin")->GetValuePose("pose"));
  }
  else
    msgs::Set(_msg.mutable_pose(), math::Pose(0, 0, 0, 0, 0, 0));

  msgs::Set(_msg.mutable_pose()->mutable_position(), this->anchorPos);


  if (this->HasType(Base::HINGE_JOINT))
    _msg.set_type(msgs::Joint::REVOLUTE);
  else if (this->HasType(Base::HINGE2_JOINT))
    _msg.set_type(msgs::Joint::REVOLUTE2);
  else if (this->HasType(Base::BALL_JOINT))
    _msg.set_type(msgs::Joint::BALL);
  else if (this->HasType(Base::SLIDER_JOINT))
    _msg.set_type(msgs::Joint::PRISMATIC);
  else if (this->HasType(Base::SCREW_JOINT))
    _msg.set_type(msgs::Joint::SCREW);
  else if (this->HasType(Base::UNIVERSAL_JOINT))
    _msg.set_type(msgs::Joint::UNIVERSAL);

  msgs::Set(_msg.mutable_axis1()->mutable_xyz(), this->GetLocalAxis(0));
  _msg.mutable_axis1()->set_limit_lower(0);
  _msg.mutable_axis1()->set_limit_upper(0);
  _msg.mutable_axis1()->set_limit_effort(0);
  _msg.mutable_axis1()->set_limit_velocity(0);
  _msg.mutable_axis1()->set_damping(0);
  _msg.mutable_axis1()->set_friction(0);

  if (this->GetParent())
    _msg.set_parent(this->GetParent()->GetScopedName());
  else
    _msg.set_parent("world");

  if (this->GetChild())
    _msg.set_child(this->GetChild()->GetScopedName());
  else
    _msg.set_child("world");
}

//////////////////////////////////////////////////
math::Angle Joint::GetAngle(int _index) const
{
  if (this->model->IsStatic())
    return this->staticAngle;
  else
    return this->GetAngleImpl(_index);
}

//////////////////////////////////////////////////
void Joint::SetAngle(int /*index*/, math::Angle _angle)
{
  this->staticAngle = _angle;
}

//////////////////////////////////////////////////
JointState Joint::GetState()
{
  return JointState(boost::shared_static_cast<Joint>(shared_from_this()));
}

//////////////////////////////////////////////////
void Joint::SetState(const JointState &_state)
{
  this->SetMaxForce(0, 0);
  this->SetVelocity(0, 0);
  for (unsigned int i = 0; i < _state.GetAngleCount(); ++i)
    this->SetAngle(i, _state.GetAngle(i));
}
