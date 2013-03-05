/*
 * Copyright 2012 Open Source Robotics Foundation
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
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Joint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Joint::Joint(BasePtr _parent)
  : Base(_parent)
{
  this->AddType(Base::JOINT);
  this->forceApplied[0] = 0;
  this->forceApplied[1] = 0;
  this->effortLimit[0] = -1;
  this->effortLimit[1] = -1;
  this->velocityLimit[0] = -1;
  this->velocityLimit[1] = -1;
}

//////////////////////////////////////////////////
Joint::~Joint()
{
}

//////////////////////////////////////////////////
void Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Vector3 &_pos)
{
  this->Load(_parent, _child, math::Pose(_pos, math::Quaternion()));
}

//////////////////////////////////////////////////
void Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Pose &_pose)
{
  if (_parent)
  {
    this->world = _parent->GetWorld();
    this->model = _parent->GetModel();
  }
  else if (_child)
  {
    this->world = _child->GetWorld();
    this->model = _child->GetModel();
  }
  else
    gzthrow("both parent and child link do no exist");

  this->parentLink = _parent;
  this->childLink = _child;

  // Joint is loaded without sdf from a model
  // Initialize this->sdf so it can be used for data storage
  sdf::initFile("joint.sdf", this->sdf);

  this->LoadImpl(_pose);
}

//////////////////////////////////////////////////
void Joint::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  std::string parentName = _sdf->GetValueString("parent");
  std::string childName = _sdf->GetValueString("child");

  if (this->model)
  {
    this->childLink = this->model->GetLink(childName);
    this->parentLink = this->model->GetLink(parentName);
  }
  else
  {
    this->childLink = boost::shared_dynamic_cast<Link>(
        this->GetWorld()->GetByName(childName));

    this->parentLink = boost::shared_dynamic_cast<Link>(
        this->GetWorld()->GetByName(parentName));
  }

  if (!this->parentLink && parentName != std::string("world"))
    gzthrow("Couldn't Find Parent Link[" + parentName + "]");

  if (!this->childLink && childName != std::string("world"))
    gzthrow("Couldn't Find Child Link[" + childName  + "]");

  this->LoadImpl(_sdf->GetValuePose("pose"));
}

/////////////////////////////////////////////////
void Joint::LoadImpl(const math::Vector3 &_pos)
{
  this->LoadImpl(math::Pose(_pos, math::Quaternion()));
}

/////////////////////////////////////////////////
void Joint::LoadImpl(const math::Pose &_pose)
{
  BasePtr myBase = shared_from_this();

  if (this->parentLink)
    this->parentLink->AddChildJoint(boost::shared_static_cast<Joint>(myBase));
  else if (this->childLink)
    this->childLink->AddParentJoint(boost::shared_static_cast<Joint>(myBase));
  else
    gzthrow("both parent and child link do no exist");

  // setting anchor relative to gazebo child link frame position
  if (this->childLink)
    this->anchorPos = (_pose + this->childLink->GetWorldPose()).pos;
  // otherwise set anchor relative to world frame
  else
    this->anchorPos = _pose.pos;
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
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(0, limitElem->GetValueDouble("upper"));
      this->SetLowStop(0, limitElem->GetValueDouble("lower"));
      this->SetHighStop(0, limitElem->GetValueDouble("upper"));
      this->effortLimit[0] = limitElem->GetValueDouble("effort");
      this->velocityLimit[0] = limitElem->GetValueDouble("velocity");
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
      this->effortLimit[1] = limitElem->GetValueDouble("effort");
      this->velocityLimit[1] = limitElem->GetValueDouble("velocity");
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
      if (this->sdf->GetValueString("parent") != "world")
      {
        gzwarn << "joint [" << this->GetScopedName()
          << "] has a non-real parentLink ["
          << this->sdf->GetValueString("parent")
          << "], loading axis from SDF ["
          << this->sdf->GetElement("axis")->GetValueVector3("xyz")
          << "]\n";
      }
    }
    if (this->sdf->HasElement("axis2"))
    {
      this->SetAxis(1, this->sdf->GetElement("axis2")->GetValueVector3("xyz"));

      if (this->sdf->GetValueString("parent") != "world")
      {
        gzwarn << "joint [" << this->GetScopedName()
          << "] has a non-real parentLink ["
          << this->sdf->GetValueString("parent")
          << "], loading axis2 from SDF ["
          << this->sdf->GetElement("axis2")->GetValueVector3("xyz")
          << "]\n";
      }
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
  // vec = this->childLink->GetWorldPose().rot.RotateVectorReverse(vec);
  // vec.Round();
  return vec;
}

//////////////////////////////////////////////////
double Joint::GetEffortLimit(unsigned int _index)
{
  if (_index < this->GetAngleCount())
    return this->effortLimit[_index];

  gzerr << "GetEffortLimit index[" << _index << "] out of range\n";
  return 0;
}

//////////////////////////////////////////////////
double Joint::GetVelocityLimit(unsigned int _index)
{
  if (_index < this->GetAngleCount())
    return this->velocityLimit[_index];

  gzerr << "GetVelocityLimit index[" << _index << "] out of range\n";
  return 0;
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
void Joint::Reset()
{
  this->SetMaxForce(0, 0);
  this->SetVelocity(0, 0);
  this->staticAngle.SetFromRadian(0);
}

//////////////////////////////////////////////////
void Joint::Attach(LinkPtr _parent, LinkPtr _child)
{
  this->parentLink = _parent;
  this->childLink = _child;
}

//////////////////////////////////////////////////
void Joint::Detach()
{
  BasePtr myBase = shared_from_this();

  if (this->parentLink)
    this->parentLink->RemoveChildJoint(
      boost::shared_static_cast<Joint>(myBase));
  this->childLink->RemoveParentJoint(boost::shared_static_cast<Joint>(myBase));
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
void Joint::FillMsg(msgs::Joint &_msg)
{
  _msg.set_name(this->GetScopedName());

  if (this->sdf->HasElement("pose"))
  {
    msgs::Set(_msg.mutable_pose(),
              this->sdf->GetValuePose("pose"));
  }
  else
    msgs::Set(_msg.mutable_pose(), math::Pose(0, 0, 0, 0, 0, 0));

  if (this->HasType(Base::HINGE_JOINT))
  {
    _msg.set_type(msgs::Joint::REVOLUTE);
    _msg.add_angle(this->GetAngle(0).Radian());
  }
  else if (this->HasType(Base::HINGE2_JOINT))
  {
    _msg.set_type(msgs::Joint::REVOLUTE2);
    _msg.add_angle(this->GetAngle(0).Radian());
    _msg.add_angle(this->GetAngle(1).Radian());
  }
  else if (this->HasType(Base::BALL_JOINT))
  {
    _msg.set_type(msgs::Joint::BALL);
  }
  else if (this->HasType(Base::SLIDER_JOINT))
  {
    _msg.set_type(msgs::Joint::PRISMATIC);
    _msg.add_angle(this->GetAngle(0).Radian());
  }
  else if (this->HasType(Base::SCREW_JOINT))
  {
    _msg.set_type(msgs::Joint::SCREW);
    _msg.add_angle(this->GetAngle(0).Radian());
  }
  else if (this->HasType(Base::UNIVERSAL_JOINT))
  {
    _msg.set_type(msgs::Joint::UNIVERSAL);
    _msg.add_angle(this->GetAngle(0).Radian());
    _msg.add_angle(this->GetAngle(1).Radian());
  }

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
void Joint::SetHighStop(int _index, const math::Angle &_angle)
{
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  if (_index == 0)
  {
    this->sdf->GetElement("axis")->GetElement("limit")
             ->GetElement("upper")->Set(_angle.Radian());
  }
  else if (_index == 1)
  {
    this->sdf->GetElement("axis2")->GetElement("limit")
             ->GetElement("upper")->Set(_angle.Radian());
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set high stop\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetLowStop(int _index, const math::Angle &_angle)
{
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  if (_index == 0)
  {
    this->sdf->GetElement("axis")->GetElement("limit")
             ->GetElement("lower")->Set(_angle.Radian());
  }
  else if (_index == 1)
  {
    this->sdf->GetElement("axis2")->GetElement("limit")
             ->GetElement("lower")->Set(_angle.Radian());
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set low stop\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetAngle(int /*index*/, math::Angle _angle)
{
  if (this->model->IsStatic())
    this->staticAngle = _angle;
  else
    this->model->SetJointPosition(this->GetScopedName(), _angle.Radian());
}

//////////////////////////////////////////////////
void Joint::SetState(const JointState &_state)
{
  this->SetMaxForce(0, 0);
  this->SetVelocity(0, 0);
  for (unsigned int i = 0; i < _state.GetAngleCount(); ++i)
    this->SetAngle(i, _state.GetAngle(i));
}

//////////////////////////////////////////////////
void Joint::SetForce(unsigned int _index, double _force)
{
  // this bit of code actually doesn't do anything physical,
  // it simply records the forces commanded inside forceApplied.
  if (_index < this->GetAngleCount())
    this->forceApplied[_index] = _force;
  else
    gzerr << "Something's wrong, joint [" << this->GetName()
          << "] index [" << _index
          << "] out of range.\n";
}

//////////////////////////////////////////////////
double Joint::GetForce(unsigned int _index)
{
  if (_index < this->GetAngleCount())
  {
    return this->forceApplied[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get force\n";
    return 0;
  }
}

//////////////////////////////////////////////////
void Joint::ApplyDamping()
{
  double dampingForce = -this->dampingCoefficient * this->GetVelocity(0);
  this->SetForce(0, dampingForce);
}
