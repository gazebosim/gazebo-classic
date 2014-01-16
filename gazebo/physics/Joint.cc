/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
  this->lowerLimit[0] = -1e16;
  this->lowerLimit[1] = -1e16;
  this->upperLimit[0] =  1e16;
  this->upperLimit[1] =  1e16;
  this->inertiaRatio[0] = 0;
  this->inertiaRatio[1] = 0;
  this->dissipationCoefficient[0] = 0;
  this->dissipationCoefficient[1] = 0;
  this->stiffnessCoefficient[0] = 0;
  this->stiffnessCoefficient[1] = 0;
  this->springReferencePosition[0] = 0;
  this->springReferencePosition[1] = 0;
  this->provideFeedback = false;
  this->stopStiffness[0] = 1e8;
  this->stopDissipation[0] = 1.0;
  this->stopStiffness[1] = 1e8;
  this->stopDissipation[1] = 1.0;
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

  this->LoadImpl(_pose);
}

//////////////////////////////////////////////////
void Joint::Load(sdf::ElementPtr _sdf)
{
  rml::Joint rmlJoint;
  rmlJoint.SetFromXML(_sdf);
  this->Load(rmlJoint);
}

//////////////////////////////////////////////////
void Joint::Load(const rml::Joint &_rml)
{
  this->rml = _rml;

  Base::Load(this->rml.name());

  // Joint force and torque feedback
  if (this->rml.has_physics())
  {
    this->SetProvideFeedback(this->rml.physics().provide_feedback());
  }

  if (this->rml.has_axis())
  {
    if (this->rml.axis().has_limit())
    {
      // store joint stop stiffness and dissipation coefficients
      this->stopStiffness[0] = this->rml.axis().limit().stiffness();
      this->stopDissipation[0] = this->rml.axis().limit().dissipation();
    }
  }

  if (this->rml.has_axis2())
  {
    if (this->rml.axis2().has_limit())
    {
      // store joint stop stiffness and dissipation coefficients
      this->stopStiffness[1] = this->rml.axis2().limit().stiffness();
      this->stopDissipation[1] = this->rml.axis2().limit().dissipation();
    }
  }

  GZ_ASSERT(this->rml.has_parent(), "Joint parent not set.");
  GZ_ASSERT(this->rml.has_child(), "Joint child not set");

  std::string parentName = this->rml.parent();
  std::string childName = this->rml.child();

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

  this->anchorPose = math::Pose(this->rml.pose());
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

  if (this->rml.has_sensor())
  {
    for (std::vector<rml::Sensor>::const_iterator iter =
         this->rml.sensor().begin(); iter != this->rml.sensor().end(); ++iter)
    {
      /// \todo This if statement is a hack to prevent Joints from creating
      /// other sensors. We should make this more generic.
      if ((*iter).type() == "force_torque")
      {
        std::string sensorName =
          sensors::create_sensor((*iter), this->GetWorld()->GetName(),
              this->GetScopedName(), this->GetId());
        this->sensors.push_back(sensorName);
      }
      else
        gzerr << "A joint cannot load a [" << (*iter).type() << "] sensor.\n";
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

  if (this->rml.has_axis())
  {
    this->SetAxis(0, math::Vector3(this->rml.axis().xyz()));
    if (this->rml.axis().has_limit())
    {
      // store upper and lower joint limits
      this->upperLimit[0] = this->rml.axis().limit().upper();
      this->lowerLimit[0] = this->rml.axis().limit().lower();

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(0, this->upperLimit[0].Radian());
      this->SetLowStop(0, this->lowerLimit[0].Radian());
      this->SetHighStop(0, this->upperLimit[0].Radian());

      this->effortLimit[0] = this->rml.axis().limit().effort();
      this->velocityLimit[0] = this->rml.axis().limit().velocity();
    }
  }

  if (this->rml.has_axis2())
  {
    this->SetAxis(1, this->rml.axis2().xyz());
    if (this->rml.axis2().has_limit())
    {

      // store upper and lower joint limits
      this->upperLimit[1] = this->rml.axis2().limit().upper();
      this->lowerLimit[1] = this->rml.axis2().limit().lower();

      // Perform this three step ordering to ensure the
      // parameters  are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(1, this->upperLimit[1].Radian());
      this->SetLowStop(1, this->lowerLimit[1].Radian());
      this->SetHighStop(1, this->upperLimit[1].Radian());

      this->effortLimit[1] = this->rml.axis2().limit().effort();
      this->velocityLimit[1] = this->rml.axis2().limit().velocity();
    }
  }

  // Set parent name: if parentLink is NULL, it's name be the world
  if (!this->parentLink)
    this->rml.set_parent("world");

  // for debugging only
  // this->ComputeInertiaRatio();
}

//////////////////////////////////////////////////
math::Vector3 Joint::GetLocalAxis(int _index) const
{
  math::Vector3 vec;

  if (_index == 0 && this->rml.has_axis())
    vec = this->rml.axis().xyz();
  else if (this->rml.has_axis2())
    vec = this->rml.axis2().xyz();
  return vec;
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
  else if (this->HasType(Base::GEARBOX_JOINT))
  {
    _msg.set_type(msgs::Joint::GEARBOX);
    _msg.add_angle(this->GetAngle(0).Radian());
    _msg.add_angle(this->GetAngle(1).Radian());
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
  if (_index == 0)
  {
    this->rml.mutable_axis()->mutable_limit()->set_upper(_angle.Radian());
  }
  else if (_index == 1)
  {
    this->rml.mutable_axis2()->mutable_limit()->set_upper(_angle.Radian());
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
  if (_index == 0)
  {
    this->rml.axis().limit().set_lower(_angle.Radian());
  }
  else if (_index == 1)
  {
    this->rml.axis2().limit().set_lower(_angle.Radian());
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
  gzerr << "Joint::GetDampingCoefficient() is deprecated, please switch "
        << "to Joint::GetDamping(index)\n";
  return this->dissipationCoefficient[0];
}

//////////////////////////////////////////////////
void Joint::ApplyDamping()
{
  gzerr << "Joint::ApplyDamping deprecated by Joint::ApplyStiffnessDamping.\n";
  this->ApplyStiffnessDamping();
}

//////////////////////////////////////////////////
void Joint::ApplyStiffnessDamping()
{
  gzerr << "Joint::ApplyStiffnessDamping should be overloaded by "
        << "physics engines.\n";
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
double Joint::GetDamping(int _index)
{
  if (static_cast<unsigned int>(_index) < this->GetAngleCount())
  {
    return this->dissipationCoefficient[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get damping coefficient.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetStiffness(unsigned int _index)
{
  if (static_cast<unsigned int>(_index) < this->GetAngleCount())
  {
    return this->stiffnessCoefficient[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get stiffness coefficient.\n";
    return 0;
  }
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
void Joint::SetLowerLimit(unsigned int _index, math::Angle _limit)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "SetLowerLimit for index [" << _index
          << "] out of bounds [" << this->GetAngleCount()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    // store lower joint limits
    this->lowerLimit[_index] = _limit;
    this->rml.axis().limit().set_lower(_limit.Radian());
  }
  else if (_index == 1)
  {
    // store lower joint limits
    this->lowerLimit[_index] = _limit;
    this->rml.axis2().limit().set_lower(_limit.Radian());
  }
  else
  {
    gzwarn << "SetLowerLimit for joint [" << this->GetName()
           << "] index [" << _index
           << "] not supported\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetUpperLimit(unsigned int _index, math::Angle _limit)
{
  if (_index >= this->GetAngleCount())
  {
    gzerr << "SetUpperLimit for index [" << _index
          << "] out of bounds [" << this->GetAngleCount()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    // store upper joint limits
    this->upperLimit[_index] = _limit;
    this->rml.axis().limit().set_upper(_limit.Radian());
  }
  else if (_index == 1)
  {
    // store upper joint limits
    this->upperLimit[_index] = _limit;
    this->rml.axis2().limit().set_upper(_limit.Radian());
  }
  else
  {
    gzwarn << "SetUpperLimit for joint [" << this->GetName()
           << "] index [" << _index
           << "] not supported\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetProvideFeedback(bool _enable)
{
  this->provideFeedback = _enable;
}

//////////////////////////////////////////////////
void Joint::SetStopStiffness(unsigned int _index, double _stiffness)
{
  if (_index < this->GetAngleCount())
  {
    this->stopStiffness[_index] = _stiffness;
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set joint stop stiffness.\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetStopDissipation(unsigned int _index, double _dissipation)
{
  if (_index < this->GetAngleCount())
  {
    this->stopDissipation[_index] = _dissipation;
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set joint stop dissipation.\n";
  }
}

//////////////////////////////////////////////////
double Joint::GetStopStiffness(unsigned int _index)
{
  if (_index < this->GetAngleCount())
  {
    return this->stopStiffness[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get joint stop stiffness.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetStopDissipation(unsigned int _index)
{
  if (_index < this->GetAngleCount())
  {
    return this->stopDissipation[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get joint stop dissipation.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
math::Pose Joint::GetInitialAnchorPose()
{
  return this->anchorPose;
}
