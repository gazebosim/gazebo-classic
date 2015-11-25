/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <boost/algorithm/string.hpp>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Joint.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ForceTorqueSensorPrivate.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("force_torque", ForceTorqueSensor)

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
: Sensor(*new ForceTorqueSensorPrivate, sensors::OTHER)
{
  this->dataPtr =
    std::static_pointer_cast<ForceTorqueSensorPrivate>(this->dPtr);
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
std::string ForceTorqueSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/wrench";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName,
                             sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  sdf::ElementPtr forceTorqueElem =
    this->dataPtr->sdf->GetElement("force_torque");

  GZ_ASSERT(forceTorqueElem,
    "force_torque element should be present in a ForceTorqueSensor sdf");

  // Handle frame setting
  ForceTorqueSensorPrivate::MeasureFrame defaultFrame =
    ForceTorqueSensorPrivate::CHILD_LINK;
  if (forceTorqueElem->HasElement("frame"))
  {
    sdf::ElementPtr frameElem = forceTorqueElem->GetElement("frame");
    std::string measureFrameSDF;
    frameElem->GetValue()->Get<std::string>(measureFrameSDF);
    if (measureFrameSDF == "parent")
    {
      this->dataPtr->measureFrame = ForceTorqueSensorPrivate::PARENT_LINK;
    }
    else if (measureFrameSDF == "child")
    {
      this->dataPtr->measureFrame = ForceTorqueSensorPrivate::CHILD_LINK;
    }
    else if (measureFrameSDF == "sensor")
    {
      this->dataPtr->measureFrame = ForceTorqueSensorPrivate::SENSOR;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque frame '"
        << measureFrameSDF << "', using default value" << std::endl;
      this->dataPtr->measureFrame = defaultFrame;
    }
  }
  else
  {
    this->dataPtr->measureFrame = defaultFrame;
  }

  // Save joint_child orientation, useful if the measure
  // is expressed in joint orientation
  GZ_ASSERT(this->dataPtr->parentJoint,
            "parentJoint should be defined by single argument Load()");
  ignition::math::Quaterniond rotationChildSensor =
    (this->dataPtr->pose +
     this->dataPtr->parentJoint->GetInitialAnchorPose().Ign()).Rot();

  this->dataPtr->rotationSensorChild = rotationChildSensor.Inverse();

  // Handle measure direction
  bool defaultDirectionIsParentToChild = false;
  if (forceTorqueElem->HasElement("measure_direction"))
  {
    sdf::ElementPtr measureDirectionElem =
      forceTorqueElem->GetElement("measure_direction");
    std::string measureDirectionSDF;
    measureDirectionElem->GetValue()->Get<std::string>(measureDirectionSDF);
    if (measureDirectionSDF == "parent_to_child")
    {
      this->dataPtr->parentToChild = true;
    }
    else if (measureDirectionSDF == "child_to_parent")
    {
      this->dataPtr->parentToChild = false;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque measure_direction \""
             << measureDirectionSDF << "\", using default value" << std::endl;
      this->dataPtr->parentToChild = defaultDirectionIsParentToChild;
    }
  }
  else
  {
    this->dataPtr->parentToChild = defaultDirectionIsParentToChild;
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  GZ_ASSERT(this->dataPtr->world != NULL,
      "ForceTorqueSensor did not get a valid World pointer");

  this->dataPtr->parentJoint = boost::dynamic_pointer_cast<physics::Joint>(
      this->dataPtr->world->GetByName(this->ParentName()));

  if (!this->dataPtr->parentJoint)
  {
    gzerr << "Parent of a force torque sensor[" << this->Name()
          << "] Must be a joint\n";
    return;
  }

  this->dataPtr->wrenchPub =
    this->dataPtr->node->Advertise<msgs::WrenchStamped>(this->Topic());
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Init()
{
  Sensor::Init();
  this->dataPtr->parentJoint->SetProvideFeedback(true);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Fini()
{
  this->dataPtr->wrenchPub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
physics::JointPtr ForceTorqueSensor::GetJoint() const
{
  return this->Joint();
}

//////////////////////////////////////////////////
physics::JointPtr ForceTorqueSensor::Joint() const
{
  return this->dataPtr->parentJoint;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ForceTorqueSensor::Force() const
{
  return msgs::ConvertIgn(this->dataPtr->wrenchMsg.wrench().force());
}

//////////////////////////////////////////////////
ignition::math::Vector3d ForceTorqueSensor::Torque() const
{
  return msgs::ConvertIgn(this->dataPtr->wrenchMsg.wrench().torque());
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->lastMeasurementTime = this->dataPtr->world->GetSimTime();
  msgs::Set(this->dataPtr->wrenchMsg.mutable_time(),
      this->dataPtr->lastMeasurementTime);

  physics::JointWrench wrench = this->dataPtr->parentJoint->GetForceTorque(0u);

  // Get the force and torque in the appropriate frame.
  ignition::math::Vector3d measuredForce;
  ignition::math::Vector3d measuredTorque;

  if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::PARENT_LINK)
  {
    if (this->dataPtr->parentToChild)
    {
      measuredForce = wrench.body1Force.Ign();
      measuredTorque = wrench.body1Torque.Ign();
    }
    else
    {
      measuredForce = -1*wrench.body1Force.Ign();
      measuredTorque = -1*wrench.body1Torque.Ign();
    }
  }
  else if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::CHILD_LINK)
  {
    if (!this->dataPtr->parentToChild)
    {
      measuredForce = wrench.body2Force.Ign();
      measuredTorque = wrench.body2Torque.Ign();
    }
    else
    {
      measuredForce = -1*wrench.body2Force.Ign();
      measuredTorque = -1*wrench.body2Torque.Ign();
    }
  }
  else
  {
    GZ_ASSERT(this->dataPtr->measureFrame == ForceTorqueSensorPrivate::SENSOR,
        "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR");

    if (!this->dataPtr->parentToChild)
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        wrench.body2Force.Ign();
      measuredTorque = this->dataPtr->rotationSensorChild *
        wrench.body2Torque.Ign();
    }
    else
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        (-1*wrench.body2Force.Ign());
      measuredTorque = this->dataPtr->rotationSensorChild *
        (-1*wrench.body2Torque.Ign());
    }
  }

  msgs::Set(this->dataPtr->wrenchMsg.mutable_wrench()->mutable_force(),
      measuredForce);
  msgs::Set(this->dataPtr->wrenchMsg.mutable_wrench()->mutable_torque(),
      measuredTorque);

  this->dataPtr->update(this->dataPtr->wrenchMsg);

  if (this->dataPtr->wrenchPub)
    this->dataPtr->wrenchPub->Publish(this->dataPtr->wrenchMsg);

  return true;
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive() const
{
  return Sensor::IsActive() || this->dataPtr->wrenchPub->HasConnections();
}

//////////////////////////////////////////////////
/*event::ConnectionPtr ForceTorqueSensor::ConnectUpdate(
    boost::function<void (msgs::WrenchStamped)> _subscriber)
{
  return this->dataPtr->update.Connect(_subscriber);
}*/

//////////////////////////////////////////////////
event::ConnectionPtr ForceTorqueSensor::ConnectUpdate(
    std::function<void (msgs::WrenchStamped)> _subscriber)
{
  return this->dataPtr->update.Connect(_subscriber);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::DisconnectUpdate(event::ConnectionPtr &_conn)
{
  this->dataPtr->update.Disconnect(_conn);
}
