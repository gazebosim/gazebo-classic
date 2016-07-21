/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/TransportIface.hh"
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

#include "gazebo/util/IntrospectionManager.hh"

using namespace gazebo;
using namespace physics;

sdf::ElementPtr Joint::sdfJoint;

//////////////////////////////////////////////////
Joint::Joint(BasePtr _parent)
: Base(*new JointPrivate, _parent),
  jointDPtr(static_cast<JointPrivate*>(this->baseDPtr))
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
Joint::Joint(BasePtr _parent)
: Base(*new JointPrivate, _parent),
  jointDPtr(static_cast<JointPrivate*>(this->baseDPtr)
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
void Joint::ConstructionHelper()
{
  this->AddType(Base::JOINT);
  this->jointDPtr->effortLimit[0] = -1;
  this->jointDPtr->effortLimit[1] = -1;
  this->jointDPtr->velocityLimit[0] = -1;
  this->jointDPtr->velocityLimit[1] = -1;
  this->jointDPtr->lowerLimit[0] = -1e16;
  this->jointDPtr->lowerLimit[1] = -1e16;
  this->jointDPtr->upperLimit[0] =  1e16;
  this->jointDPtr->upperLimit[1] =  1e16;
  this->jointDPtr->dissipationCoefficient[0] = 0;
  this->jointDPtr->dissipationCoefficient[1] = 0;
  this->jointDPtr->stiffnessCoefficient[0] = 0;
  this->jointDPtr->stiffnessCoefficient[1] = 0;
  this->jointDPtr->springReferencePosition[0] = 0;
  this->jointDPtr->springReferencePosition[1] = 0;
  this->jointDPtr->provideFeedback = false;
  this->jointDPtr->stopStiffness[0] = 1e8;
  this->jointDPtr->stopDissipation[0] = 1.0;
  this->jointDPtr->stopStiffness[1] = 1e8;
  this->jointDPtr->stopDissipation[1] = 1.0;
  // these flags are related to issue #494
  // set default to true for backward compatibility
  this->jointDPtr->axisParentModelFrame[0] = true;
  this->jointDPtr->axisParentModelFrame[1] = true;

  if (!this->jointDPtr->sdfJoint)
  {
    this->jointDPtr->sdfJoint.reset(new sdf::Element);
    sdf::initFile("joint.sdf", this->jointDPtr->sdfJoint);
  }
}

//////////////////////////////////////////////////
Joint::~Joint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Pose &_pose)
{
  this->Load(_parent, _child, _pose.Ign());
}

//////////////////////////////////////////////////
bool Joint::Load(LinkPtr _parent, LinkPtr _child,
    const ignition::math::Pose3d &_pose)
{
  if (_parent)
  {
    this->world = _parent->World();
    this->jointDPtr->model = _parent->Model();
  }
  else if (_child)
  {
    this->world = _child->World();
    this->jointDPtr->model = _child->Model();
  }
  else
  {
    gzerr << "both parent and child link do no exist\n";
    return false
  }

  this->jointDPtr->parentLink = _parent;
  this->jointDPtr->childLink = _child;

  // Joint is loaded without sdf from a model
  // Initialize this->jointDPtr->sdf so it can be used for data storage
  this->jointDPtr->sdf = this->jointDPtr->sdfJoint->Clone();

  return this->LoadImpl(_pose);
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

  for (unsigned int index = 0; index < this->AngleCount(); ++index)
  {
    std::string axisName;
    if (index == 0)
    {
      axisName = "axis";
    }
    else if (index == 1)
    {
      axisName = "axis2";
    }
    else
    {
      gzerr << "Invalid axis count" << std::endl;
      continue;
    }

    if (!_sdf->HasElement(axisName))
    {
      continue;
    }
    sdf::ElementPtr axisElem = _sdf->GetElement(axisName);
    {
      std::string param = "use_parent_model_frame";
      // Check if "use_parent_model_frame" element exists.
      // It has `required=1`, so if it does not exist, then SDF is old,
      // and we should assume support for backwards compatibility
      if (axisElem->HasElement(param))
      {
        this->jointDPtr->axisParentModelFrame[index] = axisElem->Get<bool>(param);
      }

      // Axis dynamics
      sdf::ElementPtr dynamicsElem = axisElem->GetElement("dynamics");
      if (dynamicsElem)
      {
        double reference = 0;
        double stiffness = 0;
        if (dynamicsElem->HasElement("spring_reference"))
        {
          reference = dynamicsElem->Get<double>("spring_reference");
        }
        if (dynamicsElem->HasElement("spring_stiffness"))
        {
          stiffness = dynamicsElem->Get<double>("spring_stiffness");
        }
        this->SetStiffnessDamping(index, stiffness,
            dynamicsElem->Get<double>("damping"), reference);

        if (dynamicsElem->HasElement("friction"))
        {
          this->SetParam("friction", index,
            dynamicsElem->Get<double>("friction"));
        }
      }
    }
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // store upper and lower joint limits
      this->jointDPtr->upperLimit[index] = limitElem->Get<double>("upper");
      this->jointDPtr->lowerLimit[index] = limitElem->Get<double>("lower");
      // store joint stop stiffness and dissipation coefficients
      this->jointDPtr->stopStiffness[index] = limitElem->Get<double>("stiffness");
      this->jointDPtr->stopDissipation[index] = limitElem->Get<double>("dissipation");
      // store joint effort and velocity limits
      this->jointDPtr->effortLimit[index] = limitElem->Get<double>("effort");
      this->jointDPtr->velocityLimit[index] = limitElem->Get<double>("velocity");
    }
  }

  sdf::ElementPtr parentElem = _sdf->GetElement("parent");
  sdf::ElementPtr childElem = _sdf->GetElement("child");

  GZ_ASSERT(parentElem, "Parent element is NULL");
  GZ_ASSERT(childElem, "Child element is NULL");

  std::string parentName = parentElem->Get<std::string>();
  std::string childName = childElem->Get<std::string>();

  if (this->jointDPtr->model)
  {
    this->jointDPtr->childLink = this->jointDPtr->model->GetLink(childName);
    if (!this->jointDPtr->childLink)
    {
      // need to do this if child link belongs to another model
      this->jointDPtr->childLink = std::dynamic_pointer_cast<Link>(
          this->GetWorld()->GetByName(childName));
    }
    this->jointDPtr->parentLink = this->jointDPtr->model->GetLink(parentName);
  }
  else
  {
    this->jointDPtr->childLink = std::dynamic_pointer_cast<Link>(
        this->GetWorld()->GetByName(childName));

    this->jointDPtr->parentLink = std::dynamic_pointer_cast<Link>(
        this->GetWorld()->GetByName(parentName));
  }

  // Link might not have been found because it is on another model
  // or because the model name has been changed, e.g. spawning the same model
  // twice will result in some suffix appended to the model name
  // First try to find the link with different scopes.
  if (!this->jointDPtr->parentLink && parentName != std::string("world"))
  {
    BasePtr parentModel = this->jointDPtr->model;
    while (!this->jointDPtr->parentLink && parentModel && parentModel->HasType(MODEL))
    {
      std::string scopedParentName =
          parentModel->GetScopedName() + "::" + parentName;

      this->jointDPtr->parentLink = std::dynamic_pointer_cast<Link>(
          this->GetWorld()->GetByName(scopedParentName));

      parentModel = parentModel->GetParent();
    }
    if (!this->jointDPtr->parentLink)
    {
      std::string parentNameThisModel;
      auto doubleColon = parentName.find("::");
      if (doubleColon != std::string::npos)
      {
        parentNameThisModel = parentName.substr(doubleColon);
      }
      else
      {
        parentNameThisModel = "::" + parentName;
      }
      parentNameThisModel = parentModel->GetName() + parentNameThisModel;

      this->jointDPtr->parentLink = std::dynamic_pointer_cast<Link>(
          this->GetWorld()->GetByName(parentNameThisModel));
    }
    if (!this->jointDPtr->parentLink)
      gzthrow("Couldn't Find Parent Link[" + parentName + "]");
  }

  if (!this->jointDPtr->childLink && childName != std::string("world"))
  {
    BasePtr parentModel = this->jointDPtr->model;

    while (!this->jointDPtr->childLink && parentModel && parentModel->HasType(MODEL))
    {
      std::string scopedChildName =
          parentModel->GetScopedName() + "::" + childName;
      this->jointDPtr->childLink = std::dynamic_pointer_cast<Link>(
          this->GetWorld()->GetByName(scopedChildName));

      parentModel = parentModel->GetParent();
    }
    if (!this->jointDPtr->childLink)
    {
      std::string childNameThisModel;
      auto doubleColon = childName.find("::");
      if (doubleColon != std::string::npos)
      {
        childNameThisModel = childName.substr(doubleColon);
      }
      else
      {
        childNameThisModel = "::" + childName;
      }
      childNameThisModel = parentModel->GetName() + childNameThisModel;

      this->jointDPtr->childLink = std::dynamic_pointer_cast<Link>(
          this->GetWorld()->GetByName(childNameThisModel));
    }
    if (!this->jointDPtr->childLink)
      gzthrow("Couldn't Find Child Link[" + childName  + "]");
  }

  this->LoadImpl(_sdf->Get<math::Pose>("pose"));
}

/////////////////////////////////////////////////
bool Joint::LoadImpl(const ignition::math::Pose3d &_pose)
{
  this->jointDPtr->anchorPose = _pose;

  BasePtr myBase = shared_from_this();

  if (this->jointDPtr->parentLink)
  {
    this->jointDPtr->parentLink->AddChildJoint(
        std::static_pointer_cast<Joint>(myBase));
  }

  if (this->jointDPtr->childLink)
  {
    this->jointDPtr->childLink->AddParentJoint(
        std::static_pointer_cast<Joint>(myBase));
  }

  if (!this->jointDPtr->parentLink && !this->jointDPtr->childLink)
  {
    gzerr << "both parent and child link do no exist\n";
    return false;
  }

  // setting anchor relative to gazebo child link frame position
  ignition::math::Pose3d worldPose = this->WorldPose();
  this->jointDPtr->anchorPos = worldPose.Pos();

  // Compute anchor pose relative to parent frame.
  if (this->jointDPtr->parentLink)
  {
    this->jointDPtr->parentAnchorPose = worldPose -
      this->jointDPtr->parentLink->WorldPose();
  }
  else
  {
    this->jointDPtr->parentAnchorPose = worldPose;
  }

  if (this->jointDPtr->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->jointDPtr->sdf->GetElement("sensor");
    while (sensorElem)
    {
      /// \todo This if statement is a hack to prevent Joints from creating
      /// other sensors. We should make this more generic.
      if (sensorElem->Get<std::string>("type") == "force_torque")
      {
        // This must match the implementation in Sensors::ScopedName
        std::string sensorName = this->ScopedName(true) + "::" +
          sensorElem->Get<std::string>("name");

        // Tell the sensor library to create a sensor.
        event::Events::createSensor(sensorElem,
            this->World()->Name(), this->ScopedName(), this->Id());

        this->sensors.push_back(sensorName);
      }
      else
      {
        gzerr << "A joint cannot load a [" <<
          sensorElem->Get<std::string>("type") << "] sensor.\n";
      }
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }

  return true;
}

//////////////////////////////////////////////////
void Joint::Init()
{
  try
  {
    this->Attach(this->jointDPtr->parentLink, this->jointDPtr->childLink);
  }
  catch(...)
  {
    gzerr << "Attach joint failed" << std::endl;
    return;
  }

  // Set the anchor vector
  this->SetAnchor(0, this->jointDPtr->anchorPos);

  if (this->AngleCount() >= 1 && this->jointDPtr->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis");
    this->SetAxis(0, axisElem->Get<math::Vector3>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(0, this->jointDPtr->upperLimit[0].Radian());
      this->SetLowStop(0, this->jointDPtr->lowerLimit[0].Radian());
      this->SetHighStop(0, this->jointDPtr->upperLimit[0].Radian());
    }
  }

  if (this->AngleCount() >= 2 && this->jointDPtr->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis2");
    this->SetAxis(1, axisElem->Get<math::Vector3>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters  are set properly.
      // This is taken from the ODE wiki.
      this->SetHighStop(1, this->jointDPtr->upperLimit[1].Radian());
      this->SetLowStop(1, this->jointDPtr->lowerLimit[1].Radian());
      this->SetHighStop(1, this->jointDPtr->upperLimit[1].Radian());
    }
  }

  // Set parent name: if parentLink is NULL, it's name be the world
  if (!this->jointDPtr->parentLink)
    this->jointDPtr->sdf->GetElement("parent")->Set("world");
}

//////////////////////////////////////////////////
void Joint::Fini()
{
  this->jointDPtr->applyDamping.reset();

  // Remove all the sensors attached to the joint
  for (auto const &sensor : this->jointDPtr->sensors)
  {
    event::Events::removeSensor(sensor);
  }
  this->jointDPtr->sensors.clear();

  this->anchorLink.reset();
  this->childLink.reset();
  this->parentLink.reset();
  this->model.reset();
  this->sdfJoint.reset();

  Base::Fini();
}

//////////////////////////////////////////////////
math::Vector3 Joint::GetLocalAxis(unsigned int _index) const
{
  return this->LocalAxis(_index);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Joint::LocalAxis(const unsigned int _index) const
{
  ignition::math::Vector3d vec;

  if (_index == 0 && this->jointDPtr->sdf->HasElement("axis"))
  {
    vec =this->jointDPtr->sdf->GetElement("axis")->Get<
      ignition::math::Vector3d>("xyz");
  }
  else if (this->jointDPtr->sdf->HasElement("axis2"))
  {
    vec = this->jointDPtr->sdf->GetElement("axis2")->Get<
      ignition::math::Vector3d>("xyz");
  }
  return vec;
}

//////////////////////////////////////////////////
void Joint::SetEffortLimit(const unsigned int _index, const double _effort)
{
  if (_index < this->AngleCount())
  {
    this->jointDPtr->effortLimit[_index] = _effort;
    return;
  }

  gzerr << "SetEffortLimit index[" << _index << "] out of range" << std::endl;
}

//////////////////////////////////////////////////
void Joint::SetVelocityLimit(const unsigned int _index, const double _velocity)
{
  if (_index < this->AngleCount())
  {
    this->jointDPtr->velocityLimit[_index] = _velocity;
    return;
  }

  gzerr << "SetVelocityLimit index["
        << _index
        << "] out of range"
        << std::endl;
}

//////////////////////////////////////////////////
double Joint::GetEffortLimit(unsigned int _index)
{
  return this->EffortLimit(_index);
}

//////////////////////////////////////////////////
double Joint::EffortLimit(const unsigned int _index) const
{
  if (_index < this->AngleCount())
    return this->jointDPtr->effortLimit[_index];

  gzerr << "GetEffortLimit index[" << _index << "] out of range\n";
  return 0;
}

//////////////////////////////////////////////////
double Joint::GetVelocityLimit(unsigned int _index)
{
  return this->VelocityLimit(_index);
}

//////////////////////////////////////////////////
double Joint::VelocityLimit(const unsigned int _index) const
{
  if (_index < this->AngleCount())
    return this->jointDPtr->velocityLimit[_index];

  gzerr << "GetVelocityLimit index[" << _index << "] out of range\n";
  return 0;
}

//////////////////////////////////////////////////
void Joint::Update()
{
  this->jointDPtr->jointUpdate();
}

//////////////////////////////////////////////////
void Joint::UpdateParameters(sdf::ElementPtr _sdf)
{
  Base::UpdateParameters(_sdf);
  /// \todo Update joint specific parameters. Issue #1954
}

//////////////////////////////////////////////////
void Joint::Reset()
{
  for (unsigned int i = 0; i < this->AngleCount(); ++i)
  {
    this->SetVelocity(i, 0.0);
  }
  this->jointDPtr->staticAngle.SetFromRadian(0);
}

//////////////////////////////////////////////////
void Joint::Attach(LinkPtr _parent, LinkPtr _child)
{
  this->jointDPtr->parentLink = _parent;
  this->jointDPtr->childLink = _child;
}

//////////////////////////////////////////////////
void Joint::Detach()
{
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->RemoveChildJoint(this->Name());
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->RemoveParentJoint(this->Name());
}

//////////////////////////////////////////////////
void Joint::SetModel(ModelPtr _model)
{
  this->jointDPtr->model = _model;
  this->SetWorld(this->jointDPtr->model->GetWorld());
}

//////////////////////////////////////////////////
double Joint::GetParam(const std::string &_key, unsigned int _index)
{
  return this->Param(_key, _index);
}

//////////////////////////////////////////////////
double Joint::Param(const std::string &_key, const unsigned int _index) const
{
  if (_key == "hi_stop")
  {
    return this->HighStop(_index).Radian();
  }
  else if (_key == "lo_stop")
  {
    return this->GetLowStop(_index).Radian();
  }
  gzerr << "GetParam unrecognized parameter ["
        << _key
        << "]"
        << std::endl;
  return 0;
}

//////////////////////////////////////////////////
LinkPtr Joint::GetChild() const
{
  return this->Child();
}

//////////////////////////////////////////////////
LinkPtr Joint::Child() const
{
  return this->jointDPtr->childLink;
}

//////////////////////////////////////////////////
LinkPtr Joint::GetParent() const
{
  return this->Parent();
}

//////////////////////////////////////////////////
LinkPtr Joint::Parent() const
{
  return this->jointDPtr->parentLink;
}

//////////////////////////////////////////////////
msgs::Joint::Type Joint::GetMsgType() const
{
  return this->MsgType();
}

//////////////////////////////////////////////////
msgs::Joint::Type Joint::MsgType() const
{
  if (this->HasType(Base::HINGE_JOINT))
  {
    return msgs::Joint::REVOLUTE;
  }
  else if (this->HasType(Base::HINGE2_JOINT))
  {
    return msgs::Joint::REVOLUTE2;
  }
  else if (this->HasType(Base::BALL_JOINT))
  {
    return msgs::Joint::BALL;
  }
  else if (this->HasType(Base::SLIDER_JOINT))
  {
    return msgs::Joint::PRISMATIC;
  }
  else if (this->HasType(Base::SCREW_JOINT))
  {
    return msgs::Joint::SCREW;
  }
  else if (this->HasType(Base::GEARBOX_JOINT))
  {
    return msgs::Joint::GEARBOX;
  }
  else if (this->HasType(Base::UNIVERSAL_JOINT))
  {
    return msgs::Joint::UNIVERSAL;
  }
  else if (this->HasType(Base::FIXED_JOINT))
  {
    return msgs::Joint::FIXED;
  }

  gzerr << "No joint recognized in type ["
        << this->GetType()
        << "], returning REVOLUTE"
        << std::endl;
  return msgs::Joint::REVOLUTE;
}

//////////////////////////////////////////////////
void Joint::FillMsg(msgs::Joint &_msg)
{
  _msg.set_name(this->GetScopedName());
  _msg.set_id(this->GetId());

  msgs::Set(_msg.mutable_pose(), this->jointDPtr->anchorPose.Ign());
  _msg.set_type(this->GetMsgType());

  for (unsigned int i = 0; i < this->AngleCount(); ++i)
  {
    _msg.add_angle(this->Angle(i).Radian());
    msgs::Axis *axis;
    if (i == 0)
      axis = _msg.mutable_axis1();
    else if (i == 1)
      axis = _msg.mutable_axis2();
    else
      break;

    msgs::Set(axis->mutable_xyz(), this->GetLocalAxis(i).Ign());
    axis->set_limit_lower(this->GetLowStop(i).Radian());
    axis->set_limit_upper(this->HighStop(i).Radian());
    axis->set_limit_effort(this->GetEffortLimit(i));
    axis->set_limit_velocity(this->GetVelocityLimit(i));
    axis->set_damping(this->GetDamping(i));
    axis->set_friction(this->GetParam("friction", i));
    axis->set_use_parent_model_frame(this->jointDPtr->axisParentModelFrame[i]);
  }

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

  // Add in the sensor data.
  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      msgs::Sensor *msg = _msg.add_sensor();
      msg->CopyFrom(msgs::SensorFromSDF(sensorElem));
      msg->set_parent(this->GetScopedName());
      msg->set_parent_id(this->GetId());
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }
}

//////////////////////////////////////////////////
math::Angle Joint::GetAngle(unsigned int _index) const
{
  return this->Angle(_index);
}

//////////////////////////////////////////////////
ignition::math::Angle Joint::Angle(const unsigned int _index) const
{
  if (this->jointDPtr->model->IsStatic())
    return this->jointDPtr->staticAngle;
  else
    return this->AngleImpl(_index);
}

//////////////////////////////////////////////////
bool Joint::SetHighStop(unsigned int _index, const math::Angle &_angle)
{
  return this->SetHighStop(_index, _angle.Ign());
}

//////////////////////////////////////////////////
bool Joint::SetHighStop(unsigned int _index,
    const ignition::math::Angle &_angle)
{
  this->SetUpperLimit(_index, _angle);
  // switch below to return this->SetUpperLimit when we implement
  // issue #1108
  return true;
}

//////////////////////////////////////////////////
bool Joint::SetLowStop(unsigned int _index, const math::Angle &_angle)
{
  return this->SetLowStop(_index, _angle.Ign());
}

//////////////////////////////////////////////////
bool Joint::SetLowStop(const unsigned int _index,
    const ignition::math::Angle &_angle)
{
  this->SetLowerLimit(_index, _angle);
  // switch below to return this->SetLowerLimit when we implement
  // issue #1108
  return true;
}

//////////////////////////////////////////////////
bool Joint::SetPosition(const unsigned int /*_index*/, const double _position)
{
  // parent class doesn't do much, derived classes do all the work.
  if (this->jointDPtr->model)
  {
    if (this->jointDPtr->model->IsStatic())
    {
      this->jointDPtr->staticAngle = _position;
    }
  }
  else
  {
    gzwarn << "model not setup yet, setting staticAngle.\n";
    this->jointDPtr->staticAngle = _position;
  }
  return true;
}

//////////////////////////////////////////////////
bool Joint::SetPositionMaximal(const unsigned int _index,
    const double _position)
{
  // check if index is within bounds
  if (_index >= this->AngleCount())
  {
    gzerr << "Joint axis index ["
          << _index
          << "] larger than angle count ["
          << this->AngleCount()
          << "]."
          << std::endl;
    return false;
  }

  /// If the Joint is static, Gazebo stores the state of
  /// this Joint as a scalar inside the Joint class in Joint::SetPosition.
  if (!Joint::SetPosition(_index, _position))
  {
    gzerr << "Joint::SetPosition failed, "
          << "but it's not possible as implemented.\n";
    return false;
  }

  // truncate position by joint limits
  double lower = this->LowStop(_index).Radian();
  double upper = this->HighStop(_index).Radian();
  if (lower < upper)
    _position = ignition::math::clamp(_position, lower, upper);
  else
    _position = ignition::math::clamp(_position, upper, lower);

  // only deal with hinge, universal, slider joints in the user
  // request joint_names list
  if (this->HasType(Base::HINGE_JOINT) ||
      this->HasType(Base::UNIVERSAL_JOINT) ||
      this->HasType(Base::SLIDER_JOINT))
  {
    if (childLink)
    {
      // Get all connected links to this joint
      Link_V connectedLinks;
      if (this->FindAllConnectedLinks(this->jointDPtr->parentLink,
                                      connectedLinks))
      {
        // debug
        // gzerr << "found connected links: ";
        // for (Link_V::iterator li = connectedLinks.begin();
        //                       li != connectedLinks.end(); ++li)
        //   std::cout << (*li)->GetName() << " ";
        // std::cout << "\n";

        // successfully found a subset of links connected to this joint
        // (parent link cannot be in this set).  Next, compute transform
        // to apply to all these links.

        // Everything here must be done within one time step,
        // Link pose updates need to be synchronized.

        // compute transform about the current anchor, about the axis
        // rotate child (childLink) about anchor point,

        // Get Child Link Pose
        ignition::math::Pose3d childLinkPose =
          this->jointDPtr->childLink->WorldPose();

        // Compute new child link pose based on position change
        ignition::math::Pose3d newChildLinkPose =
          this->ComputeChildLinkPose(_index, _position);

        // debug
        // gzerr << "child link pose0 [" << childLinkPose
        //       << "] new child link pose0 [" << newChildLinkPose
        //       << "]\n";

        // update all connected links
        {
          // block any other physics pose updates
          boost::recursive_mutex::scoped_lock lock(
            *this->World()->GetPhysicsEngine()->GetPhysicsUpdateMutex());

          for (Link_V::iterator li = connectedLinks.begin();
                                li != connectedLinks.end(); ++li)
          {
            // set pose of each link based on child link pose change
            (*li)->MoveFrame(childLinkPose, newChildLinkPose);

            // debug
            // gzerr << "moved " << (*li)->GetName()
            //       << " p0 [" << childLinkPose
            //       << "] p1 [" << newChildLinkPose
            //       << "]\n";
          }
        }
      }
      else
      {
        // if parent Link is found in search, return false
        gzwarn << "failed to find a clean set of connected links,"
               << " i.e. this joint is inside a loop, cannot SetPosition"
               << " kinematically.\n";
        return false;
      }
    }
    else
    {
      gzerr << "child link is null.\n";
      return false;
    }
  }
  else
  {
    gzerr << "joint type SetPosition not supported.\n";
    return false;
  }

  /// \todo:  Set link and joint "velocities" based on change / time
  return true;
}

//////////////////////////////////////////////////
bool Joint::SetVelocityMaximal(const unsigned int _index,
    const double _velocity)
{
  // check if index is within bounds
  if (_index >= this->AngleCount())
  {
    gzerr << "Joint axis index ["
          << _index
          << "] larger than angle count ["
          << this->AngleCount()
          << "]."
          << std::endl;
    return false;
  }

  // Set child link relative to parent for now.
  // TODO: recursive velocity setting on trees.
  if (!this->jointDPtr->childLink)
  {
    gzerr << "SetVelocityMaximal failed for joint ["
          << this->ScopedName()
          << "] since a child link was not found."
          << std::endl;
    return false;
  }

  // only deal with hinge, universal, slider joints for now
  if (this->HasType(Base::HINGE_JOINT) ||
      this->HasType(Base::UNIVERSAL_JOINT))
  {
    // Desired angular and linear velocity in world frame for child link
    ignition::math::Vector3d angularVel, linearVel;
    if (this->jointDPtr->parentLink)
    {
      // Use parent link velocity as reference (if parent exists)
      angularVel = this->jointDPtr->parentLink->WorldAngularVel();

      // Get parent linear velocity at joint anchor
      // Passing unit quaternion q ensures that parentOffset will be
      //  interpreted in world frame.
      ignition::math::Quaterniond q;
      ignition::math::Vector3d parentOffset =
        this->ParentWorldPose().Pos() -
        this->jointDPtr->parentLink->WorldPose().Pos();
      linearVel = this->jointDPtr->parentLink->WorldLinearVel(parentOffset, q);
    }

    // Add desired velocity along specified axis
    angularVel += _velocity * this->GlobalAxis(_index);

    if (this->HasType(Base::UNIVERSAL_JOINT))
    {
      // For multi-axis joints, retain velocity of other axis.
      unsigned int otherIndex = (_index + 1) % 2;
      angularVel += this->Velocity(otherIndex) * this->GlobalAxis(otherIndex);
    }

    this->jointDPtr->childLink->SetAngularVel(angularVel);

    // Compute desired linear velocity of the child link based on
    //  offset between the child's CG and the joint anchor
    //  and the desired angular velocity.
    ignition::math::Vector3d childCoGOffset =
      this->jointDPtr->childLink->WorldCoGPose().Pos() -
      this->WorldPose().Pos();
    linearVel += angularVel.Cross(childCoGOffset);
    this->jointDPtr->childLink->SetLinearVel(linearVel);
  }
  else if (this->HasType(Base::SLIDER_JOINT))
  {
    ignition::math::Vector3d desiredVel;
    if (this->jointDPtr->parentLink)
    {
      desiredVel = this->jointDPtr->parentLink->WorldLinearVel();
    }
    desiredVel += _velocity * this->GlobalAxis(_index);
    this->jointDPtr->childLink->SetLinearVel(desiredVel);
  }
  else
  {
    gzerr << "SetVelocityMaximal does not yet support"
          << " this joint type."
          << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void Joint::SetState(const JointState &_state)
{
  for (unsigned int i = 0; i < _state.GetAngleCount(); ++i)
  {
    this->SetVelocity(i, 0.0);
    this->SetPosition(i, _state.GetAngle(i).Radian());
  }
}

//////////////////////////////////////////////////
double Joint::CheckAndTruncateForce(const unsigned int _index,
    const double _effort)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Calling Joint::SetForce with an index ["
          << _index << "] out of range\n";
    return _effort;
  }

  // truncating SetForce effort if velocity limit is reached and
  // effort is applied in the same direction.
  if (this->jointDPtr->velocityLimit[_index] >= 0)
  {
    if (this->Velocity(_index) > this->jointDPtr->velocityLimit[_index])
    {
      _effort = _effort > 0 ? 0 : _effort;
    }
    else if (this->GetVelocity(_index) <
             -this->jointDPtr->velocityLimit[_index])
    {
      _effort = _effort < 0 ? 0 : _effort;
    }
  }

  // truncate effort if effortLimit is not negative
  if (this->jointDPtr->effortLimit[_index] >= 0.0)
  {
    _effort = math::clamp(_effort, -this->jointDPtr->effortLimit[_index],
      this->jointDPtr->effortLimit[_index]);
  }

  return _effort;
}

//////////////////////////////////////////////////
double Joint::GetForce(unsigned int _index)
{
  return this->Force(_index);
}

//////////////////////////////////////////////////
double Joint::Force(const unsigned int /*_index*/) const
{
  gzerr << "Joint::Force should be overloaded by physics engines.\n";
  return 0;
}

//////////////////////////////////////////////////
void Joint::ApplyStiffnessDamping()
{
  gzerr << "Joint::ApplyStiffnessDamping should be overloaded by "
        << "physics engines.\n";
}

//////////////////////////////////////////////////
double Joint::GetInertiaRatio(const math::Vector3 &_axis) const
{
  return this->InertiaRatio(_axis.Ign());
}

//////////////////////////////////////////////////
double Joint::GetInertiaRatio(const ignition::math::Vector3d &_axis) const
{
  if (this->jointDPtr->parentLink && this->jointDPtr->childLink)
  {
    ignition::math::Matrix3d pm =
      this->jointDPtr->parentLink->WorldInertiaMatrix();
    ignition::math::Matrix3d cm =
      this->jointDPtr->childLink->WorldInertiaMatrix();

    // matrix times axis
    ignition::math::Vector3 pia = pm * _axis;
    ignition::math::Vector3 cia = cm * _axis;
    double piam = pia.Length();
    double ciam = cia.Length();

    // return ratio of child MOI to parent MOI.
    if (!ignition::math::equal(piam, 0.0))
    {
      return ciam/piam;
    }
    else
    {
      gzerr << "Parent MOI is zero, ratio is not well defined.\n";
      return 0;
    }
  }
  else
  {
    gzerr << "Either parent or child link is missing or static, "
          << "cannot compute inertia ratio.  Returning 0.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetInertiaRatio(const unsigned int _index) const
{
  return this->InertiaRatio(_index);
}

//////////////////////////////////////////////////
double Joint::InertiaRatio(const unsigned int _index) const
{
  if (this->jointDPtr->parentLink && this->jointDPtr->childLink)
  {
    if (_index < this->AngleCount())
    {
      // joint axis in global frame
      ignition::math::Vector3d axis = this->GlobalAxis(_index);

      // compute ratio about axis
      return this->InertiaRatio(axis);
    }
    else
    {
      gzerr << "Invalid joint index [" << _index
            << "] when trying to get inertia ratio across joint.\n";
      return 0;
    }
  }
  else
  {
    gzerr << "Either parent or child link is missing or static, "
          << "cannot compute inertia ratio.  Returning 0.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetDamping(unsigned int _index)
{
  return this->Damping(_index);
}

//////////////////////////////////////////////////
double Joint::Damping(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->jointDPtr->dissipationCoefficient[_index];
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
  return this->SpringStiffness(_index);
}

//////////////////////////////////////////////////
double Joint::Stiffness(const unsigned int _index) const
{
  if (static_cast<unsigned int>(_index) < this->AngleCount())
  {
    return this->jointDPtr->stiffnessCoefficient[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get stiffness coefficient.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetSpringReferencePosition(unsigned int _index) const
{
  return this->SpringReferencePosition(_index);
}

//////////////////////////////////////////////////
double Joint::SpringReferencePosition(unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->jointDPtr->springReferencePosition[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get spring reference position.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
math::Angle Joint::GetLowerLimit(unsigned int _index) const
{
  return this->LowerLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Angle Joint::LowerLimit(const unsigned int _index) const
{
  if (_index < this->AngleCount())
    return this->jointDPtr->lowerLimit[_index];

  gzwarn << "requesting lower limit of joint index out of bound\n";
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
math::Angle Joint::GetUpperLimit(unsigned int _index) const
{
  return this->UpperLimit(_index);
}

//////////////////////////////////////////////////
ignition::math::Angle Joint::UpperLimit(const unsigned int _index) const
{
  if (_index < this->AngleCount())
    return this->jointDPtr->upperLimit[_index];

  gzwarn << "requesting upper limit of joint index out of bound\n";
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
void Joint::SetLowerLimit(unsigned int _index, math::Angle _limit)
{
  return this->SetLowerLimit(_index, _limit.Ign());
}

//////////////////////////////////////////////////
void Joint::LowerLimit(const unsigned int _index,
                       const ignition::math::Angle &_limit)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "SetLowerLimit for index [" << _index
          << "] out of bounds [" << this->AngleCount()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store lower joint limits
    this->jointDPtr->lowerLimit[_index] = _limit;
    limitElem->GetElement("lower")->Set(_limit.Radian());
  }
  else if (_index == 1)
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis2");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store lower joint limits
    this->jointDPtr->lowerLimit[_index] = _limit;
    limitElem->GetElement("lower")->Set(_limit.Radian());
  }
  else
  {
    gzwarn << "SetLowerLimit for joint [" << this->Name()
           << "] index [" << _index
           << "] not supported\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetUpperLimit(unsigned int _index, math::Angle _limit)
{
  return this->SetUpperLimit(_index, _limit);
}

//////////////////////////////////////////////////
void Joint::SetUpperLimit(const unsigned int _index,
    const ignition::math::Angle &_limit)
{
  if (_index >= this->AngleCount())
  {
    gzerr << "SetUpperLimit for index [" << _index
          << "] out of bounds [" << this->AngleCount()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store upper joint limits
    this->jointDPtr->upperLimit[_index] = _limit;
    limitElem->GetElement("upper")->Set(_limit.Radian());
  }
  else if (_index == 1)
  {
    sdf::ElementPtr axisElem = this->jointDPtr->sdf->GetElement("axis2");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store upper joint limits
    this->jointDPtr->upperLimit[_index] = _limit;
    limitElem->GetElement("upper")->Set(_limit.Radian());
  }
  else
  {
    gzwarn << "SetUpperLimit for joint [" << this->Name()
           << "] index [" << _index
           << "] not supported\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetProvideFeedback(const bool _enable)
{
  this->jointDPtr->provideFeedback = _enable;
}

//////////////////////////////////////////////////
void Joint::SetStopStiffness(const unsigned int _index, const double _stiffness)
{
  if (_index < this->AngleCount())
  {
    this->jointDPtr->stopStiffness[_index] = _stiffness;
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set joint stop stiffness.\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetStopDissipation(const unsigned int _index,
    const double _dissipation)
{
  if (_index < this->AngleCount())
  {
    this->jointDPtr->stopDissipation[_index] = _dissipation;
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to set joint stop dissipation.\n";
  }
}

//////////////////////////////////////////////////
double Joint::GetStopStiffness(unsigned int _index) const
{
  return this->StopStiffness(_index);
}

//////////////////////////////////////////////////
double Joint::StopStiffness(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->jointDPtr->stopStiffness[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get joint stop stiffness.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::GetStopDissipation(unsigned int _index) const
{
  return this->StopDissipation(_index);
}

//////////////////////////////////////////////////
double Joint::StopDissipation(const unsigned int _index) const
{
  if (_index < this->AngleCount())
  {
    return this->jointDPtr->stopDissipation[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get joint stop dissipation.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
math::Pose Joint::GetInitialAnchorPose() const
{
  return this->InitialAnchorPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::InitialAnchorPose() const
{
  return this->jointDPtr->anchorPose;
}

//////////////////////////////////////////////////
math::Pose Joint::GetWorldPose() const
{
  return this->WorldPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::WorldPose() const
{
  if (this->jointDPtr->childLink)
  {
    return this->jointDPtr->anchorPose +
      this->jointDPtr->childLink->GetWorldPose();
  }
  return this->jointDPtr->anchorPose;
}

//////////////////////////////////////////////////
math::Pose Joint::GetParentWorldPose() const
{
  return this->ParentWorldPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::ParentWorldPose() const
{
  if (this->jointDPtr->parentLink)
  {
    return this->jointDPtr->parentAnchorPose +
      this->jointDPtr->parentLink->WorldPose();
  }
  return this->jointDPtr->parentAnchorPose;
}

//////////////////////////////////////////////////
math::Pose Joint::GetAnchorErrorPose() const
{
  return this->AnchorErrorPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::AnchorErrorPose() const
{
  return this->WorldPose() - this->ParentWorldPose();
}

//////////////////////////////////////////////////
math::Quaternion Joint::GetAxisFrame(unsigned int _index) const
{
  return this->AxisFrame(_index);
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Joint::AxisFrame(const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "GetAxisFrame error, _index[" << _index << "] out of range"
          << std::endl;
    return ignition::math::Quaternion();
  }

  // Legacy support for specifying axis in parent model frame (#494)
  if (this->jointDPtr->axisParentModelFrame[_index])
  {
    // Use parent model frame
    if (this->jointDPtr->parentLink)
      return this->jointDPtr->parentLink->GetModel()->GetWorldPose().rot;

    // Parent model is world, use world frame
    return ignition::math::Quaternion();
  }

  return this->WorldPose().Rot();
}

//////////////////////////////////////////////////
math::Quaternion Joint::GetAxisFrameOffset(unsigned int _index) const
{
  return this->AxisFrameOffset(_index);
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Joint::AxisFrameOffset(
    const unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "GetAxisFrame error, _index[" << _index << "] out of range"
          << " returning identity rotation." << std::endl;
    return igniton::math::Quaterniond();
  }

  // Legacy support for specifying axis in parent model frame (#494)
  if (this->jointDPtr->axisParentModelFrame[_index])
  {
    // axis is defined in parent model frame, so return the rotation
    // from joint frame to parent model frame, or
    // world frame in absence of parent link.
    ignition::math::Pose3d parentModelWorldPose;
    ignition::math::Pose3d jointWorldPose = this->WorldPose();
    if (this->jointDPtr->parentLink)
    {
      parentModelWorldPose = this->jointDPtr->parentLink->Model()->WorldPose();
    }
    return (parentModelWorldPose - jointWorldPose).Rot();
  }

  // axis is defined in the joint frame, so
  // return the rotation from joint frame to joint frame.
  return ignition::math::Quaterniond();
}

//////////////////////////////////////////////////
double Joint::GetWorldEnergyPotentialSpring(unsigned int _index) const
{
  return this->WorldEnergyPotentialSpring(_index);
}

//////////////////////////////////////////////////
double Joint::WorldEnergyPotentialSpring(unsigned int _index) const
{
  if (_index >= this->AngleCount())
  {
    gzerr << "Get spring potential error, _index[" << _index
          << "] out of range" << std::endl;
    return 0;
  }

  // compute potential energy due to spring compression
  // 1/2 k x^2
  double k = this->jointDPtr->stiffnessCoefficient[_index];
  double x = this->Angle(_index).Radian() -
    this->jointDPtr->springReferencePosition[_index];
  return 0.5 * k * x * x;
}

//////////////////////////////////////////////////
void Joint::CacheForceTorque()
{
}

//////////////////////////////////////////////////
bool Joint::FindAllConnectedLinks(const LinkPtr &_originalParentLink,
  Link_V &_connectedLinks)
{
  // debug
  // std::string pn;
  // if (_originalParentLink) pn = _originalParentLink->GetName();
  // gzerr << "first call to find connected links: "
  //       << " parent " << pn
  //       << " this joint " << this->Name() << "\n";

  // unlikely, but check anyways to make sure we don't have a 0-height tree
  if (this->jointDPtr->childLink.get() == _originalParentLink.get())
  {
    // if parent is a child
    gzerr << "we have a zero length loop.\n";
    _connectedLinks.clear();
    return false;
  }
  else
  {
    // add this->jointDPtr->childLink to the list of descendent child links (should be
    // the very first one added).
    _connectedLinks.push_back(this->jointDPtr->childLink);

    // START RECURSIVE SEARCH, start adding child links of this->jointDPtr->childLink
    // to the collection of _connectedLinks.
    return this->jointDPtr->childLink->FindAllConnectedLinksHelper(_originalParentLink,
      _connectedLinks, true);
  }
}

//////////////////////////////////////////////////
math::Pose Joint::ComputeChildLinkPose(unsigned int _index,
          double _position)
{
  return this->ChildLinkPose(_index, _position);
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::ChildLinkPose(const unsigned int _index,
          const double _position)
{
  // child link pose
  ignition::math::Pose3d childLinkPose =
    this->jointDPtr->childLink->WorldPose();

  // default return to current pose
  ignition::math::Pose3d newRelativePose;
  ignition::math::Pose3d newWorldPose = childLinkPose;

  // get anchor and axis of the joint
  ignition::math::Vector3d anchor;
  ignition::math::Vector3d axis;

  if (this->jointDPtr->model->IsStatic())
  {
    /// \TODO: we want to get axis in global frame, but GetGlobalAxis
    /// not implemented for static models yet.
    axis = childLinkPose.Rot().RotateVector(this->LocalAxis(_index));
    anchor = childLinkPose.Pos();
  }
  else
  {
    anchor = this->Anchor(_index);
    axis = this->lobalAxis(_index);
  }

  // delta-position along an axis
  double dposition = _position - this->Angle(_index).Radian();

  if (this->HasType(Base::HINGE_JOINT) ||
      this->HasType(Base::UNIVERSAL_JOINT))
  {
    // relative to anchor point
    ignition::math::Pose3d relativePose(childLinkPose.Pos() - anchor,
                                        childLinkPose.Rot());

    // take axis rotation and turn it into a quaternion
    ignition::math::Quaterniond rotation(axis, dposition);

    // rotate relative pose by rotation

    newRelativePose.Pos() = rotation.RotateVector(relativePose.Pos());
    newRelativePose.Rot() = rotation * relativePose.Rot();

    newWorldPose = ignition::math::Pose3d(newRelativePose.Pos() + anchor,
        newRelativePose.Rot());

    // \TODO: ideally we want to set this according to
    // Joint Trajectory velocity and use time step since last update.
    /*
    double dt =
      this->jointDPtr->model->GetWorld()->GetPhysicsEngine()->GetMaxStepTime();
    this->ComputeAndSetLinkTwist(_link, newWorldPose, newWorldPose, dt);
    */
  }
  else if (this->HasType(Base::SLIDER_JOINT))
  {
    // relative to anchor point
    ignition::math::Pose3d relativePose(childLinkPose.Pos() - anchor,
                                        childLinkPose.Rot());

    // slide relative pose by dposition along axis
    newRelativePose.Pos() = relativePose.Pos() + axis * dposition;
    newRelativePose.Rot() = relativePose.Rot();

    newWorldPose = ignition::math::Pose3d(newRelativePose.pos + anchor,
        newRelativePose.Rot());

    /// \TODO: ideally we want to set this according to Joint Trajectory
    /// velocity and use time step since last update.
    /*
    double dt =
      this->jointDPtr->model->GetWorld()->GetPhysicsEngine()->GetMaxStepTime();
    this->ComputeAndSetLinkTwist(_link, newWorldPose, newWorldPose, dt);
    */
  }
  else
  {
    gzerr << "Setting joint position is only supported for"
          << " hinge, universal and slider joints right now.\n";
  }

  return newWorldPose;
}

/////////////////////////////////////////////////
LinkPtr Joint::GetJointLink(unsigned int _index) const
{
  return this->JointLink(_index);
}

/////////////////////////////////////////////////
void Joint::SetAxis(unsigned int _index,
    const math::Vector3 &_axis)
{
  return this->SetAxis(_index, _axis.Ign());
}

/////////////////////////////////////////////////
event::ConnectionPtr Joint::ConnectJointUpdate(
    std::function<void()> _subscriber)
{
  return this->jointDPtr->jointUpdate.Connect(_subscriber);
}

/////////////////////////////////////////////////
void Joint::DisconnectJointUpdate(event::ConnectionPtr &_conn)
{
  this->jointDPtr->jointUpdate.Disconnect(_conn);
}

/////////////////////////////////////////////////
math::Vector3 Joint::GetGlobalAxis(unsigned int _index) const
{
  return this->GlobalAxis(_index);
}

/////////////////////////////////////////////////
void Joint::SetAnchor(unsigned int _index,
    const math::Vector3 &_anchor)
{
  return this->SetAnchor(_index, _anchor.Ign());
}

/////////////////////////////////////////////////
math::Vector3 Joint::GetAnchor(unsigned int _index) const
{
  return this->Anchor(_index);
}

/////////////////////////////////////////////////
math::Angle Joint::GetHighStop(unsigned int _index)
{
  return this->HighStop(_index);
}

/////////////////////////////////////////////////
math::Angle Joint::GetLowStop(unsigned int _index)
{
  return this->LowStop(_index);
}

/////////////////////////////////////////////////
double Joint::GetVelocity(unsigned int _index) const
{
  return this->Velocity(_index);
}

/////////////////////////////////////////////////
JointWrench Joint::GetForceTorque(unsigned int _index)
{
  return this->ForceTorque(_index);
}

/////////////////////////////////////////////////
unsigned int Joint::GetAngleCount() const
{
  return this->AngleCount();
}

/////////////////////////////////////////////////
math::Vector3 Joint::GetLinkForce(unsigned int _index) const
{
  return this->LinkForce(_index);
}

/////////////////////////////////////////////////
math::Vector3 Joint::GetLinkTorque(unsigned int _index) const
{
  return this->LinkTorque(_index);
}

/////////////////////////////////////////////////
math::Angle Joint::GetAngleImpl(unsigned int _index) const
{
  return this->AngleImpl(_index);
}

/////////////////////////////////////////////////
void Joint::SetAxis(const unsigned int _index)
{
  // record axis in sdf element
  if (_index == 0)
    this->jointDPtr->sdf->GetElement("axis")->GetElement("xyz")->Set(_axis);
  else if (_index == 1)
    this->jointDPtr->sdf->GetElement("axis2")->GetElement("xyz")->Set(_axis);
  else
    gzerr << "SetAxis index [" << _index << "] out of bounds\n";
}

/////////////////////////////////////////////////
void Joint::RegisterIntrospectionItems()
{
  for (size_t i = 0; i < this->GetAngleCount(); ++i)
  {
    this->RegisterIntrospectionPosition(i);
    this->RegisterIntrospectionVelocity(i);
  }
}

/////////////////////////////////////////////////
void Joint::RegisterIntrospectionPosition(const unsigned int _index)
{
  auto f = [this, _index]()
  {
    // For prismatic axes, Radian -> meters
    return this->GetAngle(_index).Ign().Radian();
  };

  common::URI uri(this->URI());
  uri.Query().Insert("p",
      "axis/" + std::to_string(_index) + "/double/position");
  this->introspectionItems.push_back(uri);
  gazebo::util::IntrospectionManager::Instance()->Register
      <double>(uri.Str(), f);
}

/////////////////////////////////////////////////
void Joint::RegisterIntrospectionVelocity(const unsigned int _index)
{
  auto f = [this, _index]()
  {
    return this->GetVelocity(_index);
  };

  common::URI uri(this->URI());
  uri.Query().Insert("p",
      "axis/" + std::to_string(_index) + "/double/velocity");
  this->introspectionItems.push_back(uri);
  gazebo::util::IntrospectionManager::Instance()->Register
      <double>(uri.Str(), f);
}
