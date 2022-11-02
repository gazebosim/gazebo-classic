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
#include <boost/algorithm/string.hpp>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

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
: Sensor(sensors::OTHER),
  dataPtr(new ForceTorqueSensorPrivate)
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
  this->Fini();
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
    this->sdf->GetElement("force_torque");

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
    (this->pose +
     this->dataPtr->parentJoint->InitialAnchorPose()).Rot();

  this->dataPtr->rotationSensorChild =
    ignition::math::Matrix3d(rotationChildSensor.Inverse());

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
  GZ_ASSERT(this->world != nullptr,
      "ForceTorqueSensor did not get a valid World pointer");

  this->dataPtr->parentJoint = boost::dynamic_pointer_cast<physics::Joint>(
      this->world->BaseByName(this->ParentName()));

  if (!this->dataPtr->parentJoint)
  {
    gzerr << "Parent of a force torque sensor[" << this->Name()
          << "] Must be a joint\n";
    return;
  }

  this->dataPtr->wrenchPub =
    this->node->Advertise<msgs::WrenchStamped>(this->Topic());
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
  this->dataPtr->parentJoint.reset();

  Sensor::Fini();
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
bool ForceTorqueSensor::UpdateImpl(const bool /*_force*/)
{
  IGN_PROFILE("ForceTorqueSensor::UpdateImpl");
  IGN_PROFILE_BEGIN("Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->lastMeasurementTime = this->world->SimTime();
  msgs::Set(this->dataPtr->wrenchMsg.mutable_time(),
      this->lastMeasurementTime);

  physics::JointWrench wrench = this->dataPtr->parentJoint->GetForceTorque(0u);

  // Get the force and torque in the appropriate frame.
  ignition::math::Vector3d measuredForce;
  ignition::math::Vector3d measuredTorque;

  if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::PARENT_LINK)
  {
    if (this->dataPtr->parentToChild)
    {
      measuredForce = wrench.body1Force;
      measuredTorque = wrench.body1Torque;
    }
    else
    {
      measuredForce = -1*wrench.body1Force;
      measuredTorque = -1*wrench.body1Torque;
    }
  }
  else if (this->dataPtr->measureFrame == ForceTorqueSensorPrivate::CHILD_LINK)
  {
    if (!this->dataPtr->parentToChild)
    {
      measuredForce = wrench.body2Force;
      measuredTorque = wrench.body2Torque;
    }
    else
    {
      measuredForce = -1*wrench.body2Force;
      measuredTorque = -1*wrench.body2Torque;
    }
  }
  else
  {
    GZ_ASSERT(this->dataPtr->measureFrame == ForceTorqueSensorPrivate::SENSOR,
        "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR");

    if (!this->dataPtr->parentToChild)
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        wrench.body2Force;
      measuredTorque = this->dataPtr->rotationSensorChild *
        wrench.body2Torque;
    }
    else
    {
      measuredForce = this->dataPtr->rotationSensorChild *
        (-1*wrench.body2Force);
      measuredTorque = this->dataPtr->rotationSensorChild *
        (-1*wrench.body2Torque);
    }
  }

  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("Publish");

  msgs::Set(this->dataPtr->wrenchMsg.mutable_wrench()->mutable_force(),
      measuredForce);
  msgs::Set(this->dataPtr->wrenchMsg.mutable_wrench()->mutable_torque(),
      measuredTorque);

  this->dataPtr->update(this->dataPtr->wrenchMsg);

  if (this->dataPtr->wrenchPub)
    this->dataPtr->wrenchPub->Publish(this->dataPtr->wrenchMsg);
  IGN_PROFILE_END();

  return true;
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive() const
{
  return Sensor::IsActive() || this->dataPtr->wrenchPub->HasConnections();
}

//////////////////////////////////////////////////
event::ConnectionPtr ForceTorqueSensor::ConnectUpdate(
    std::function<void (msgs::WrenchStamped)> _subscriber)
{
  return this->dataPtr->update.Connect(_subscriber);
}
