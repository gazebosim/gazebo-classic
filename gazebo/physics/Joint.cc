/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
  // these flags are related to issue #494
  // set default to true for backward compatibility
  this->axisParentModelFrame[0] = true;
  this->axisParentModelFrame[1] = true;

  if (!this->sdfJoint)
  {
    this->sdfJoint.reset(new sdf::Element);
    sdf::initFile("joint.sdf", this->sdfJoint);
  }
}

//////////////////////////////////////////////////
Joint::~Joint()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Joint::Load(LinkPtr _parent, LinkPtr _child,
                 const ignition::math::Pose3d &_pose)
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

  for (unsigned int index = 0; index < this->DOF(); ++index)
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
        this->axisParentModelFrame[index] = axisElem->Get<bool>(param);
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
      this->upperLimit[index] = limitElem->Get<double>("upper");
      this->lowerLimit[index] = limitElem->Get<double>("lower");
      // store joint stop stiffness and dissipation coefficients
      this->stopStiffness[index] = limitElem->Get<double>("stiffness");
      this->stopDissipation[index] = limitElem->Get<double>("dissipation");
      // store joint effort and velocity limits
      this->effortLimit[index] = limitElem->Get<double>("effort");
      this->velocityLimit[index] = limitElem->Get<double>("velocity");
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
    if (!this->childLink)
    {
      // need to do this if child link belongs to another model
      this->childLink = boost::dynamic_pointer_cast<Link>(
          this->GetWorld()->BaseByName(childName));
    }
    this->parentLink = this->model->GetLink(parentName);
  }
  else
  {
    this->childLink = boost::dynamic_pointer_cast<Link>(
        this->GetWorld()->BaseByName(childName));

    this->parentLink = boost::dynamic_pointer_cast<Link>(
        this->GetWorld()->BaseByName(parentName));
  }

  // Link might not have been found because it is on another model
  // or because the model name has been changed, e.g. spawning the same model
  // twice will result in some suffix appended to the model name
  // First try to find the link with different scopes.
  if (!this->parentLink && parentName != std::string("world"))
  {
    BasePtr parentModel = this->model;
    while (!this->parentLink && parentModel && parentModel->HasType(MODEL))
    {
      std::string scopedParentName =
          parentModel->GetScopedName() + "::" + parentName;

      this->parentLink = boost::dynamic_pointer_cast<Link>(
          this->GetWorld()->BaseByName(scopedParentName));

      parentModel = parentModel->GetParent();
    }
    if (!this->parentLink)
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

      this->parentLink = boost::dynamic_pointer_cast<Link>(
          this->GetWorld()->BaseByName(parentNameThisModel));
    }
    if (!this->parentLink)
      gzthrow("Couldn't Find Parent Link[" + parentName + "]");
  }

  if (!this->childLink && childName != std::string("world"))
  {
    BasePtr parentModel = this->model;

    while (!this->childLink && parentModel && parentModel->HasType(MODEL))
    {
      std::string scopedChildName =
          parentModel->GetScopedName() + "::" + childName;
      this->childLink = boost::dynamic_pointer_cast<Link>(
          this->GetWorld()->BaseByName(scopedChildName));

      parentModel = parentModel->GetParent();
    }
    if (!this->childLink)
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

      this->childLink = boost::dynamic_pointer_cast<Link>(
          this->GetWorld()->BaseByName(childNameThisModel));
    }
    if (!this->childLink)
      gzthrow("Couldn't Find Child Link[" + childName  + "]");
  }

  this->LoadImpl(_sdf->Get<ignition::math::Pose3d>("pose"));
}

/////////////////////////////////////////////////
void Joint::LoadImpl(const ignition::math::Pose3d &_pose)
{
  this->anchorPose = _pose;

  BasePtr myBase = shared_from_this();

  if (this->parentLink)
    this->parentLink->AddChildJoint(boost::static_pointer_cast<Joint>(myBase));

  if (this->childLink)
    this->childLink->AddParentJoint(boost::static_pointer_cast<Joint>(myBase));

  if (!this->parentLink && !this->childLink)
    gzthrow("both parent and child link do no exist");

  // setting anchor relative to gazebo child link frame position
  auto worldPose = this->WorldPose();
  this->anchorPos = worldPose.Pos();

  // Compute anchor pose relative to parent frame.
  if (this->parentLink)
    this->parentAnchorPose = worldPose - this->parentLink->WorldPose();
  else
    this->parentAnchorPose = worldPose;

  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      /// \todo This if statement is a hack to prevent Joints from creating
      /// other sensors. We should make this more generic.
      if (sensorElem->Get<std::string>("type") == "force_torque")
      {
        // This must match the implementation in Sensors::GetScopedName
        std::string sensorName = this->GetScopedName(true) + "::" +
          sensorElem->Get<std::string>("name");

        // Tell the sensor library to create a sensor.
        event::Events::createSensor(sensorElem,
            this->GetWorld()->Name(), this->GetScopedName(), this->GetId());

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

  if (this->DOF() >= 1 && this->sdf->HasElement("axis"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    this->SetAxis(0, axisElem->Get<ignition::math::Vector3d>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters are set properly.
      // This is taken from the ODE wiki.
      this->SetUpperLimit(0, this->upperLimit[0]);
      this->SetLowerLimit(0, this->lowerLimit[0]);
      this->SetUpperLimit(0, this->upperLimit[0]);
    }
  }

  if (this->DOF() >= 2 && this->sdf->HasElement("axis2"))
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    this->SetAxis(1, axisElem->Get<ignition::math::Vector3d>("xyz"));
    if (axisElem->HasElement("limit"))
    {
      sdf::ElementPtr limitElem = axisElem->GetElement("limit");

      // Perform this three step ordering to ensure the
      // parameters  are set properly.
      // This is taken from the ODE wiki.
      this->SetUpperLimit(1, this->upperLimit[1]);
      this->SetLowerLimit(1, this->lowerLimit[1]);
      this->SetUpperLimit(1, this->upperLimit[1]);
    }
  }

  // Set parent name: if parentLink is NULL, it's name be the world
  if (!this->parentLink)
    this->sdf->GetElement("parent")->Set("world");
}

//////////////////////////////////////////////////
void Joint::Fini()
{
  this->applyDamping.reset();

  // Remove all the sensors attached to the joint
  for (auto const &sensor : this->sensors)
  {
    event::Events::removeSensor(sensor);
  }
  this->sensors.clear();

  this->anchorLink.reset();
  this->childLink.reset();
  this->parentLink.reset();
  this->model.reset();
  this->sdfJoint.reset();

  Base::Fini();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Joint::LocalAxis(const unsigned int _index) const
{
  ignition::math::Vector3d vec;

  if (_index == 0 && this->sdf->HasElement("axis"))
    vec = this->sdf->GetElement("axis")->Get<ignition::math::Vector3d>("xyz");
  else if (this->sdf->HasElement("axis2"))
    vec = this->sdf->GetElement("axis2")->Get<ignition::math::Vector3d>("xyz");
  // vec = this->childLink->GetWorldPose().Rot().RotateVectorReverse(vec);
  // vec.Round();
  return vec;
}

//////////////////////////////////////////////////
void Joint::SetEffortLimit(unsigned int _index, double _effort)
{
  if (_index < this->DOF())
  {
    this->effortLimit[_index] = _effort;
    return;
  }

  gzerr << "SetEffortLimit index[" << _index << "] out of range" << std::endl;
}

//////////////////////////////////////////////////
void Joint::SetVelocityLimit(unsigned int _index, double _velocity)
{
  if (_index < this->DOF())
  {
    this->velocityLimit[_index] = _velocity;
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
  if (_index < this->DOF())
    return this->effortLimit[_index];

  gzerr << "GetEffortLimit index[" << _index << "] out of range\n";
  return 0;
}

//////////////////////////////////////////////////
double Joint::GetVelocityLimit(unsigned int _index)
{
  if (_index < this->DOF())
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
  /// \todo Update joint specific parameters. Issue #1954
}

//////////////////////////////////////////////////
void Joint::Reset()
{
  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    this->SetVelocity(i, 0.0);
  }
  this->staticPosition = 0.0;
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
double Joint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key == "hi_stop")
  {
    return this->UpperLimit(_index);
  }
  else if (_key == "lo_stop")
  {
    return this->LowerLimit(_index);
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
  return this->childLink;
}

//////////////////////////////////////////////////
LinkPtr Joint::GetParent() const
{
  return this->parentLink;
}

//////////////////////////////////////////////////
msgs::Joint::Type Joint::GetMsgType() const
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

  msgs::Set(_msg.mutable_pose(), this->anchorPose);
  _msg.set_type(this->GetMsgType());

  for (unsigned int i = 0; i < this->DOF(); ++i)
  {
    _msg.add_angle(this->Position(i));
    msgs::Axis *axis;
    if (i == 0)
      axis = _msg.mutable_axis1();
    else if (i == 1)
      axis = _msg.mutable_axis2();
    else
      break;

    msgs::Set(axis->mutable_xyz(), this->LocalAxis(i));
    axis->set_limit_lower(this->LowerLimit(i));
    axis->set_limit_upper(this->UpperLimit(i));
    axis->set_limit_effort(this->GetEffortLimit(i));
    axis->set_limit_velocity(this->GetVelocityLimit(i));
    axis->set_damping(this->GetDamping(i));
    axis->set_friction(this->GetParam("friction", i));
    axis->set_use_parent_model_frame(this->axisParentModelFrame[i]);
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
double Joint::Position(const unsigned int _index) const
{
  if (this->model->IsStatic())
    return this->staticPosition;
  else
    return this->PositionImpl(_index);
}

//////////////////////////////////////////////////
bool Joint::SetPosition(const unsigned int /*_index*/, const double _position,
                        const bool /*_preserveWorldVelocity*/)
{
  // parent class doesn't do much, derived classes do all the work.
  if (this->model)
  {
    if (this->model->IsStatic())
    {
      this->staticPosition = _position;
    }
  }
  else
  {
    gzwarn << "model not setup yet, setting staticPosition.\n";
    this->staticPosition = _position;
  }
  return true;
}

//////////////////////////////////////////////////
bool Joint::SetPositionMaximal(
    const unsigned int _index, double _position,
    const bool _preserveWorldVelocity)
{
  // check if index is within bounds
  if (_index >= this->DOF())
  {
    gzerr << "Joint axis index ["
          << _index
          << "] larger than DOF count ["
          << this->DOF()
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
  double lower = this->LowerLimit(_index);
  double upper = this->UpperLimit(_index);
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
      if (this->FindAllConnectedLinks(this->parentLink, connectedLinks))
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
        auto childLinkPose = this->childLink->WorldPose();

        // Compute new child link pose based on position change
        ignition::math::Pose3d newChildLinkPose =
          this->ChildLinkPose(_index, _position);

        // debug
        // gzerr << "child link pose0 [" << childLinkPose
        //       << "] new child link pose0 [" << newChildLinkPose
        //       << "]\n";

        // update all connected links
        {
          // block any other physics pose updates
          boost::recursive_mutex::scoped_lock lock(
            *this->GetWorld()->Physics()->GetPhysicsUpdateMutex());

          for (Link_V::iterator li = connectedLinks.begin();
                                li != connectedLinks.end(); ++li)
          {
            // set pose of each link based on child link pose change
            (*li)->MoveFrame(childLinkPose, newChildLinkPose,
              _preserveWorldVelocity);

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
bool Joint::SetVelocityMaximal(unsigned int _index, double _velocity)
{
  // check if index is within bounds
  if (_index >= this->DOF())
  {
    gzerr << "Joint axis index ["
          << _index
          << "] larger than DOF count ["
          << this->DOF()
          << "]."
          << std::endl;
    return false;
  }

  // Set child link relative to parent for now.
  // TODO: recursive velocity setting on trees.
  if (!this->childLink)
  {
    gzerr << "SetVelocityMaximal failed for joint ["
          << this->GetScopedName()
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
    if (this->parentLink)
    {
      // Use parent link velocity as reference (if parent exists)
      angularVel = this->parentLink->WorldAngularVel();

      // Get parent linear velocity at joint anchor
      // Passing unit quaternion q ensures that parentOffset will be
      //  interpreted in world frame.
      ignition::math::Quaterniond q;
      ignition::math::Vector3d parentOffset =
        this->ParentWorldPose().Pos() - this->parentLink->WorldPose().Pos();
      linearVel = this->parentLink->WorldLinearVel(parentOffset, q);
    }

    // Add desired velocity along specified axis
    angularVel += _velocity * this->GlobalAxis(_index);

    if (this->HasType(Base::UNIVERSAL_JOINT))
    {
      // For multi-axis joints, retain velocity of other axis.
      unsigned int otherIndex = (_index + 1) % 2;
      angularVel += this->GetVelocity(otherIndex)
                  * this->GlobalAxis(otherIndex);
    }

    this->childLink->SetAngularVel(angularVel);

    // Compute desired linear velocity of the child link based on
    //  offset between the child's CG and the joint anchor
    //  and the desired angular velocity.
    ignition::math::Vector3d childCoGOffset =
      this->childLink->WorldCoGPose().Pos() -
      this->WorldPose().Pos();
    linearVel += angularVel.Cross(childCoGOffset);
    this->childLink->SetLinearVel(linearVel);
  }
  else if (this->HasType(Base::SLIDER_JOINT))
  {
    ignition::math::Vector3d desiredVel;
    if (this->parentLink)
    {
      desiredVel = this->parentLink->WorldLinearVel();
    }
    desiredVel += _velocity * this->GlobalAxis(_index);
    this->childLink->SetLinearVel(desiredVel);
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
    this->SetPosition(i, _state.Position(i));
  }
}

//////////////////////////////////////////////////
double Joint::CheckAndTruncateForce(unsigned int _index, double _effort)
{
  if (_index >= this->DOF())
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
    _effort = ignition::math::clamp(_effort, -this->effortLimit[_index],
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
void Joint::ApplyStiffnessDamping()
{
  gzerr << "Joint::ApplyStiffnessDamping should be overloaded by "
        << "physics engines.\n";
}

/////////////////////////////////////////////////
double Joint::InertiaRatio(const ignition::math::Vector3d &_axis) const
{
  if (this->parentLink && this->childLink)
  {
    auto pm = this->parentLink->WorldInertiaMatrix();
    auto cm = this->childLink->WorldInertiaMatrix();

    // matrix times axis
    ignition::math::Vector3d pia = pm * _axis;
    ignition::math::Vector3d cia = cm * _axis;
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
  if (this->parentLink && this->childLink)
  {
    if (_index < this->DOF())
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
  if (_index < this->DOF())
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
  if (static_cast<unsigned int>(_index) < this->DOF())
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
double Joint::GetSpringReferencePosition(unsigned int _index) const
{
  if (_index < this->DOF())
  {
    return this->springReferencePosition[_index];
  }
  else
  {
    gzerr << "Invalid joint index [" << _index
          << "] when trying to get spring reference position.\n";
    return 0;
  }
}

//////////////////////////////////////////////////
double Joint::LowerLimit(const unsigned int _index) const
{
  if (_index < this->DOF())
    return this->lowerLimit[_index];

  gzwarn << "requesting lower limit of joint index out of bound\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double Joint::UpperLimit(const unsigned int _index) const
{
  if (_index < this->DOF())
    return this->upperLimit[_index];

  gzwarn << "requesting upper limit of joint index out of bound\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void Joint::SetLowerLimit(const unsigned int _index, const double _limit)
{
  if (_index >= this->DOF())
  {
    gzerr << "SetLowerLimit for index [" << _index
          << "] out of bounds [" << this->DOF()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store lower joint limits
    this->lowerLimit[_index] = _limit;
    limitElem->GetElement("lower")->Set(_limit);
  }
  else if (_index == 1)
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store lower joint limits
    this->lowerLimit[_index] = _limit;
    limitElem->GetElement("lower")->Set(_limit);
  }
  else
  {
    gzwarn << "SetLowerLimit for joint [" << this->GetName()
           << "] index [" << _index
           << "] not supported\n";
  }
}

//////////////////////////////////////////////////
void Joint::SetUpperLimit(const unsigned int _index, const double _limit)
{
  if (_index >= this->DOF())
  {
    gzerr << "SetUpperLimit for index [" << _index
          << "] out of bounds [" << this->DOF()
          << "]\n";
    return;
  }

  if (_index == 0)
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store upper joint limits
    this->upperLimit[_index] = _limit;
    limitElem->GetElement("upper")->Set(_limit);
  }
  else if (_index == 1)
  {
    sdf::ElementPtr axisElem = this->sdf->GetElement("axis2");
    sdf::ElementPtr limitElem = axisElem->GetElement("limit");

    // store upper joint limits
    this->upperLimit[_index] = _limit;
    limitElem->GetElement("upper")->Set(_limit);
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
  if (_index < this->DOF())
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
  if (_index < this->DOF())
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
double Joint::GetStopStiffness(unsigned int _index) const
{
  if (_index < this->DOF())
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
double Joint::GetStopDissipation(unsigned int _index) const
{
  if (_index < this->DOF())
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
ignition::math::Pose3d Joint::InitialAnchorPose() const
{
  return this->anchorPose;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::WorldPose() const
{
  if (this->childLink)
    return this->anchorPose + this->childLink->WorldPose();
  return this->anchorPose;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::ParentWorldPose() const
{
  if (this->parentLink)
    return this->parentAnchorPose + this->parentLink->WorldPose();
  return this->parentAnchorPose;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::AnchorErrorPose() const
{
  return this->WorldPose() - this->ParentWorldPose();
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Joint::AxisFrame(const unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "AxisFrame error, _index[" << _index << "] out of range"
          << std::endl;
    return ignition::math::Quaterniond::Identity;
  }

  // Legacy support for specifying axis in parent model frame (#494)
  if (this->axisParentModelFrame[_index])
  {
    // Use parent model frame
    if (this->parentLink)
      return this->parentLink->GetModel()->WorldPose().Rot();

    // Parent model is world, use world frame
    return ignition::math::Quaterniond::Identity;
  }

  return this->WorldPose().Rot();
}

//////////////////////////////////////////////////
ignition::math::Quaterniond Joint::AxisFrameOffset(
    const unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "AxisFrameOffset error, _index[" << _index << "] out of range"
          << " returning identity rotation." << std::endl;
    return ignition::math::Quaterniond::Identity;
  }

  // Legacy support for specifying axis in parent model frame (#494)
  if (this->axisParentModelFrame[_index])
  {
    // axis is defined in parent model frame, so return the rotation
    // from joint frame to parent model frame, or
    // world frame in absence of parent link.
    ignition::math::Pose3d parentModelWorldPose;
    auto jointWorldPose = this->WorldPose();
    if (this->parentLink)
    {
      parentModelWorldPose = this->parentLink->GetModel()->WorldPose();
    }
    return (parentModelWorldPose - jointWorldPose).Rot();
  }

  // axis is defined in the joint frame, so
  // return the rotation from joint frame to joint frame.
  return ignition::math::Quaterniond::Identity;
}

//////////////////////////////////////////////////
double Joint::GetWorldEnergyPotentialSpring(unsigned int _index) const
{
  if (_index >= this->DOF())
  {
    gzerr << "Get spring potential error, _index[" << _index
          << "] out of range" << std::endl;
    return 0;
  }

  // compute potential energy due to spring compression
  // 1/2 k x^2
  double k = this->stiffnessCoefficient[_index];
  double x = this->Position(_index) -
    this->springReferencePosition[_index];
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
  //       << " this joint " << this->GetName() << "\n";

  // unlikely, but check anyways to make sure we don't have a 0-height tree
  if (this->childLink.get() == _originalParentLink.get())
  {
    // if parent is a child
    gzerr << "we have a zero length loop.\n";
    _connectedLinks.clear();
    return false;
  }
  else
  {
    // add this->childLink to the list of descendent child links (should be
    // the very first one added).
    _connectedLinks.push_back(this->childLink);

    // START RECURSIVE SEARCH, start adding child links of this->childLink
    // to the collection of _connectedLinks.
    return this->childLink->FindAllConnectedLinksHelper(_originalParentLink,
      _connectedLinks, true);
  }
}

//////////////////////////////////////////////////
ignition::math::Pose3d Joint::ChildLinkPose(const unsigned int _index,
    const double _position)
{
  // child link pose
  auto childLinkPose = this->childLink->WorldPose();

  // default return to current pose
  ignition::math::Pose3d newRelativePose;
  ignition::math::Pose3d newWorldPose = childLinkPose;

  // get anchor and axis of the joint
  ignition::math::Vector3d anchor;
  ignition::math::Vector3d axis;

  if (this->model->IsStatic())
  {
    /// \TODO: we want to get axis in global frame, but GlobalAxis
    /// not implemented for static models yet.
    axis = childLinkPose.Rot().RotateVector(this->LocalAxis(_index));
    anchor = childLinkPose.Pos();
  }
  else
  {
    anchor = this->Anchor(_index);
    axis = this->GlobalAxis(_index);
  }

  // delta-position along an axis
  double dposition = _position - this->Position(_index);

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
      this->dataPtr->model->GetWorld()-Physics()->GetMaxStepTime();
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

    newWorldPose = ignition::math::Pose3d(newRelativePose.Pos() + anchor,
        newRelativePose.Rot());

    /// \TODO: ideally we want to set this according to Joint Trajectory
    /// velocity and use time step since last update.
    /*
    double dt =
      this->dataPtr->model->GetWorld()-Physics()->GetMaxStepTime();
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
void Joint::RegisterIntrospectionItems()
{
  for (size_t i = 0; i < this->DOF(); ++i)
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
    return this->Position(_index);
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

