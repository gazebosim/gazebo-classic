/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/SensorsIface.hh"

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

sdf::ElementPtr Joint::sdfJoint;

//////////////////////////////////////////////////
Joint::Joint(BasePtr _parent)
  : Base(_parent)
{
  this->AddType(Base::JOINT);
  this->effortLimit[0] = -1;
  this->effortLimit[1] = -1;
  this->velocityLimit[0] = -1;
  this->velocityLimit[1] = -1;
  this->useCFMDamping = false;
  this->lowerLimit[0] = -1e16;
  this->lowerLimit[1] = -1e16;
  this->upperLimit[0] =  1e16;
  this->upperLimit[1] =  1e16;
  this->inertiaRatio[0] = 0;
  this->inertiaRatio[1] = 0;
  this->dampingCoefficient = 0;
  this->provideFeedback = false;

  if (!this->sdfJoint)
  {
    this->sdfJoint.reset(new sdf::Element);
    sdf::initFile("joint.sdf", this->sdfJoint);
  }
}

//////////////////////////////////////////////////
Joint::~Joint()
{
  for (std::vector<std::string>::iterator iter = this->sensors.begin();
      iter != this->sensors.end(); ++iter)
  {
    sensors::remove_sensor(*iter);
  }
  this->sensors.clear();
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
  this->sdf = this->sdfJoint->Clone();

  this->LoadImpl(_pose);
}

//////////////////////////////////////////////////
void Joint::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);

  // Joint force and torque feedback
  if (_sdf->HasElement("physics"))
  {
    sdf::ElementPtr physicsElem = _sdf->GetElement("physics");
    if (physicsElem->HasElement("provide_feedback"))
    {
      this->SetProvideFeedback(physicsElem->Get<bool>("provide_feedback"));
    }
  }

  sdf::ElementPtr parentElem = _sdf->GetElement("parent");
  sdf::ElementPtr childElem = _sdf->GetElement("child");

  GZ_ASSERT(parentElem, "Parent element is NULL");
  GZ_ASSERT(childElem, "Child element is NULL");

  std::string parentName = parentElem->Get<std::string>();
  std::string childName = childElem->Get<std::string>();

  if (this->model)
  {
    this->childLink = this->model->GetLink(childName);
    this->parentLink = this->model->GetLink(parentName);
  }
  else
  {
    this->childLink = boost::dynamic_pointer_cast<Link>(
        this->GetWorld()->GetByName(childName));

    this->parentLink = boost::dynamic_pointer_cast<Link>(
        this->GetWorld()->GetByName(parentName));
  }

  if (!this->parentLink && parentName != std::string("world"))
    gzthrow("Couldn't Find Parent Link[" + parentName + "]");

  if (!this->childLink && childName != std::string("world"))
    gzthrow("Couldn't Find Child Link[" + childName  + "]");

  this->anchorPose = _sdf->Get<math::Pose>("pose");
  this->LoadImpl(this->anchorPose);
}

/////////////////////////////////////////////////
void Joint::LoadImpl(const math::Pose &_pose)
{
  BasePtr myBase = shared_from_this();

  if (this->parentLink)
    this->parentLink->AddChildJoint(boost::static_pointer_cast<Joint>(myBase));

  if (this->childLink)
    this->childLink->AddParentJoint(boost::static_pointer_cast<Joint>(myBase));

  if (!this->parentLink && !this->childLink)
    gzthrow("both parent and child link do no exist");

  // setting anchor relative to gazebo child link frame position
  if (this->childLink)
    this->anchorPos = (_pose + this->childLink->GetWorldPose()).pos;
  // otherwise set anchor relative to world frame
  else
    this->anchorPos = _pose.pos;

  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      /// \todo This if statement is a hack to prevent Joints from creating
      /// other sensors. We should make this more generic.
      if (sensorElem->Get<std::string>("type") == "force_torque")
      {
        std::string sensorName =
          sensors::create_sensor(sensorElem, this->GetWorld()->GetName(),
              this->GetScopedName(), this->GetId());
        this->sensors.push_back(sensorName);
      }
      else
        gzerr << "A joint cannot load a [" <<
          sensorElem->Get<std::string>("type") << "] sensor.\n";
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }
}

//////////////////////////////////////////////////
void Joint::Init()
{
  try
  {
    this->Attach(this->parentLink, this->childLink);
  }
  catch(...)
  {
    gzerr << "Attach joint failed" << std::endl;
    return;
  }

  // Set the anchor vector
  this->SetAnchor(0, this->anchorPos);

  if (this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    this->SetAxis(0, axisElem->Get<math::Vector3>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // store upper and lower joint limits
      this->upperLimit[0] = limitElem->Get<double>("upper");
      this->lowerLimit[0] = limitElem->Get<double>("lower");

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(0, this->upperLimit[0].Radian());
      this->SetLowStop(0, this->lowerLimit[0].Radian());
      this->SetHighStop(0, this->upperLimit[0].Radian());

      this->effortLimit[0] = limitElem->Get<double>("effort");
      this->velocityLimit[0] = limitElem->Get<double>("velocity");
    }
  }

  if (this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    this->SetAxis(1, axisElem->Get<math::Vector3>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // store upper and lower joint limits
      this->upperLimit[1] = limitElem->Get<double>("upper");
      this->lowerLimit[1] = limitElem->Get<double>("lower");

      // Perform this three step ordering to ensure the
      // parameters  are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(1, this->upperLimit[1].Radian());
      this->SetLowStop(1, this->lowerLimit[1].Radian());
      this->SetHighStop(1, this->upperLimit[1].Radian());

      this->effortLimit[1] = limitElem->Get<double>("effort");
      this->velocityLimit[1] = limitElem->Get<double>("velocity");
    }
  }

  // Set parent name: if parentLink is NULL, it's name be the world
  if (!this->parentLink)
    this->sdf->GetElement("parent")->Set("world");

  // for debugging only
  // this->ComputeInertiaRatio();
}

//////////////////////////////////////////////////
math::Vector3 Joint::GetLocalAxis(int _index) const
{
  math::Vector3 vec;

  if (_index == 0 && this->sdf->HasElement("axis"))
    vec = this->sdf->GetElement("axis")->Get<math::Vector3>("xyz");
  else if (this->sdf->HasElement("axis2"))
    vec = this->sdf->GetElement("axis2")->Get<math::Vector3>("xyz");
  // vec = this->childLink->GetWorldPose().rot.RotateVectorReverse(vec);
  // vec.Round();
  return vec;
}

//////////////////////////////////////////////////
void Joint::SetEffortLimit(unsigned int _index, double _effort)
{
  if (_index < this->GetAngleCount())
  {
    this->effortLimit[_index] = _effort;
    return;
  }

  gzerr << "SetEffortLimit index[" << _index << "] out of range" << std::endl;
}

//////////////////////////////////////////////////
double Joint::GetEffortLimit(int _index)
{
  if (_index >= 0 && static_cast<unsigned int>(_index) < this->GetAngleCount())
    return this->effortLimit[_index];

  gzerr << "GetEffortLimit index[" << _index << "] out of range\n";
  return 0;
}

//////////////////////////////////////////////////
double Joint::GetVelocityLimit(int _index)
{
  if (_index >= 0 && static_cast<unsigned int>(_index) < this->GetAngleCount())
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
  if (this->parentLink)
    this->parentLink->RemoveChildJoint(this->GetName());
  if (this->childLink)
    this->childLink->RemoveParentJoint(this->GetName());
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
  _msg.set_id(this->GetId());

  msgs::Set(_msg.mutable_pose(), this->anchorPose);

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
  {
    _msg.set_parent(this->GetParent()->GetScopedName());
    _msg.set_parent_id(this->GetParent()->GetId());
  }
  else
  {
    _msg.set_parent("world");
    _msg.set_parent_id(0);
  }

  if (this->GetChild())
  {
    _msg.set_child(this->GetChild()->GetScopedName());
    _msg.set_child_id(this->GetChild()->GetId());
  }
  else
  {
    _msg.set_child("world");
    _msg.set_parent_id(0);
  }

  for (std::vector<std::string>::iterator iter = this->sensors.begin();
       iter != this->sensors.end(); ++iter)
  {
    sensors::SensorPtr sensor = sensors::get_sensor(*iter);
    if (sensor)
    {
      msgs::Sensor *sensorMsg =_msg.add_sensor();
      sensor->FillMsg(*sensorMsg);
    }
    else
    {
      gzlog << "Joint::FillMsg: sensor [" << *iter << "] not found.\n";
    }
  }
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
void Joint::SetAngle(int _index, math::Angle _angle)
{
  if (this->model->IsStatic())
    this->staticAngle = _angle;
  else
    this->model->SetJointPosition(
      this->GetScopedName(), _angle.Radian(), _index);
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
double Joint::CheckAndTruncateForce(int _index, double _effort)
{
  if (_index < 0 || static_cast<unsigned int>(_index) >= this->GetAngleCount())
  {
    gzerr << "Calling Joint::SetForce with an index ["
          << _index << "] out of range\n";
    return _effort;
  }

  // truncating SetForce effort if velocity limit is reached and
  // effort is applied in the same direction.
  if (this->velocityLimit[_index] >= 0)
  {
    if (this->GetVelocity(_index) > this->velocityLimit[_index])
      _effort = _effort > 0 ? 0 : _effort;
    else if (this->GetVelocity(_index) < -this->velocityLimit[_index])
      _effort = _effort < 0 ? 0 : _effort;
  }

  // truncate effort if effortLimit is not negative
  if (this->effortLimit[_index] >= 0.0)
    _effort = math::clamp(_effort, -this->effortLimit[_index],
      this->effortLimit[_index]);

  return _effort;
}

//////////////////////////////////////////////////
double Joint::GetForce(unsigned int /*_index*/)
{
  gzerr << "Joint::GetForce should be overloaded by physics engines.\n";
  return 0;
}

//////////////////////////////////////////////////
double Joint::GetDampingCoefficient() const
{
  return this->dampingCoefficient;
}

//////////////////////////////////////////////////
void Joint::ApplyDamping()
{
  gzerr << "Joint::ApplyDamping should be overloaded by physics engines.\n";
}

//////////////////////////////////////////////////
void Joint::ComputeInertiaRatio()
{
  for (unsigned int i = 0; i < this->GetAngleCount(); ++i)
  {
    math::Vector3 axis = this->GetGlobalAxis(i);
    if (this->parentLink && this->childLink)
    {
      physics::InertialPtr pi = this->parentLink->GetInertial();
      physics::InertialPtr ci = this->childLink->GetInertial();
      math::Matrix3 pm(
       pi->GetIXX(), pi->GetIXY(), pi->GetIXZ(),
       pi->GetIXY(), pi->GetIYY(), pi->GetIYZ(),
       pi->GetIXZ(), pi->GetIYZ(), pi->GetIZZ());
      math::Matrix3 cm(
       ci->GetIXX(), ci->GetIXY(), ci->GetIXZ(),
       ci->GetIXY(), ci->GetIYY(), ci->GetIYZ(),
       ci->GetIXZ(), ci->GetIYZ(), ci->GetIZZ());

      // rotate pm and cm into inertia frame
      math::Pose pPose = this->parentLink->GetWorldPose();
      math::Pose cPose = this->childLink->GetWorldPose();
      for (unsigned col = 0; col < 3; ++col)
      {
        // get each column, and inverse rotate by pose
        math::Vector3 pmCol(pm[0][col], pm[1][col], pm[2][col]);
        pmCol = pPose.rot.RotateVector(pmCol);
        pm.SetCol(col, pmCol);
        math::Vector3 cmCol(cm[0][col], cm[1][col], cm[2][col]);
        cmCol = pPose.rot.RotateVector(cmCol);
        cm.SetCol(col, cmCol);
      }

      // matrix times axis
      // \todo: add operator in Matrix3 class so we can do Matrix3 * Vector3
      math::Vector3 pia(
        pm[0][0] * axis.x + pm[0][1] * axis.y + pm[0][2] * axis.z,
        pm[1][0] * axis.x + pm[1][1] * axis.y + pm[1][2] * axis.z,
        pm[2][0] * axis.x + pm[2][1] * axis.y + pm[2][2] * axis.z);
      math::Vector3 cia(
        cm[0][0] * axis.x + cm[0][1] * axis.y + cm[0][2] * axis.z,
        cm[1][0] * axis.x + cm[1][1] * axis.y + cm[1][2] * axis.z,
        cm[2][0] * axis.x + cm[2][1] * axis.y + cm[2][2] * axis.z);
      double piam = pia.GetLength();
      double ciam = cia.GetLength();

      // should we flip? sure, so the measure of ratio is between [1, +inf]
      if (piam > ciam)
        this->inertiaRatio[i] = piam/ciam;
      else
        this->inertiaRatio[i] = ciam/piam;
    }
  }
}

//////////////////////////////////////////////////
double Joint::GetInertiaRatio(unsigned int _index) const
{
  if (_index < this->GetAngleCount())
  {
    return this->inertiaRatio[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get inertia ratio across joint.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetDamping(int /*_index*/)
{
  return this->dampingCoefficient;
}

//////////////////////////////////////////////////
math::Angle Joint::GetLowerLimit(unsigned int _index) const
{
  if (_index < this->GetAngleCount())
    return this->lowerLimit[_index];

  gzwarn << "requesting lower limit of joint index out of bound\n";
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle Joint::GetUpperLimit(unsigned int _index) const
{
  if (_index < this->GetAngleCount())
    return this->upperLimit[_index];

  gzwarn << "requesting upper limit of joint index out of bound\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void Joint::SetProvideFeedback(bool _enable)
{
  this->provideFeedback = _enable;
}

//////////////////////////////////////////////////
math::Pose Joint::GetInitialAnchorPose()
{
  return this->anchorPose;
}
