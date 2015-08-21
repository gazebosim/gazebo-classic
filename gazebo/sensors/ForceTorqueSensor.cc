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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Joint.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("force_torque", ForceTorqueSensor)

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
: Sensor(sensors::OTHER)
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
std::string ForceTorqueSensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/wrench";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName,
                             sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  sdf::ElementPtr forceTorqueElem = this->sdf->GetElement("force_torque");

  GZ_ASSERT(forceTorqueElem,
    "force_torque element should be present in a ForceTorqueSensor sdf");

  // Handle frame setting
  MeasureFrame defaultFrame = CHILD_LINK;
  if (forceTorqueElem->HasElement("frame"))
  {
    sdf::ElementPtr frameElem = forceTorqueElem->GetElement("frame");
    std::string measureFrameSDF;
    frameElem->GetValue()->Get<std::string>(measureFrameSDF);
    if (measureFrameSDF == "parent")
    {
      this->measureFrame = PARENT_LINK;
    }
    else if (measureFrameSDF == "child")
    {
      this->measureFrame = CHILD_LINK;
    }
    else if (measureFrameSDF == "sensor")
    {
      this->measureFrame = SENSOR;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque frame \"" << measureFrameSDF
             << "\", using default value" << std::endl;
      this->measureFrame = defaultFrame;
    }
  }
  else
  {
    this->measureFrame = defaultFrame;
  }

  // Save joint_child orientation, useful if the measure
  // is expressed in joint orientation
  GZ_ASSERT(this->parentJoint,
            "parentJoint should be defined by single argument Load()");
  ignition::math::Quaterniond rotationChildSensor =
    (this->pose + this->parentJoint->GetInitialAnchorPose().Ign()).Rot();
  this->rotationSensorChild = rotationChildSensor.Inverse();

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
      this->parentToChild = true;
    }
    else if (measureDirectionSDF == "child_to_parent")
    {
      this->parentToChild = false;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque measure_direction \""
             << measureDirectionSDF << "\", using default value" << std::endl;
      this->parentToChild = defaultDirectionIsParentToChild;
    }
  }
  else
  {
    this->parentToChild = defaultDirectionIsParentToChild;
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  GZ_ASSERT(this->world != NULL,
      "ForceTorqueSensor did not get a valid World pointer");

  this->parentJoint = boost::dynamic_pointer_cast<physics::Joint>(
      this->world->GetByName(this->parentName));

  if (!this->parentJoint)
  {
    gzerr << "Parent of a force torque sensor[" << this->GetName()
          << "] Must be a joint\n";
    return;
  }

  this->wrenchPub = this->node->Advertise<msgs::WrenchStamped>(
      this->GetTopic());
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Init()
{
  Sensor::Init();
  this->parentJoint->SetProvideFeedback(true);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Fini()
{
  this->wrenchPub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
physics::JointPtr ForceTorqueSensor::GetJoint() const
{
  return this->parentJoint;
}

//////////////////////////////////////////////////
math::Vector3 ForceTorqueSensor::GetForce() const
{
  return this->Force();
}

//////////////////////////////////////////////////
ignition::math::Vector3d ForceTorqueSensor::Force() const
{
  return msgs::ConvertIgn(this->wrenchMsg.wrench().force());
}

//////////////////////////////////////////////////
math::Vector3 ForceTorqueSensor::GetTorque() const
{
  return this->Torque();
}

//////////////////////////////////////////////////
ignition::math::Vector3d ForceTorqueSensor::Torque() const
{
  return msgs::ConvertIgn(this->wrenchMsg.wrench().torque());
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->wrenchMsg.mutable_time(), this->lastMeasurementTime);

  physics::JointWrench wrench = this->parentJoint->GetForceTorque(0u);

  // Get the force and torque in the appropriate frame.
  ignition::math::Vector3d measuredForce;
  ignition::math::Vector3d measuredTorque;

  if (this->measureFrame == PARENT_LINK)
  {
    if (this->parentToChild)
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
  else if (this->measureFrame == CHILD_LINK)
  {
    if (!this->parentToChild)
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
    GZ_ASSERT(this->measureFrame == SENSOR,
              "measureFrame must be PARENT_LINK, CHILD_LINK or SENSOR");
    if (!this->parentToChild)
    {
      measuredForce = rotationSensorChild*wrench.body2Force.Ign();
      measuredTorque = rotationSensorChild*wrench.body2Torque.Ign();
    }
    else
    {
      measuredForce = rotationSensorChild*(-1*wrench.body2Force.Ign());
      measuredTorque = rotationSensorChild*(-1*wrench.body2Torque.Ign());
    }
  }

  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_force(),
      measuredForce);
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_torque(),
      measuredTorque);

  this->update(this->wrenchMsg);

  if (this->wrenchPub)
    this->wrenchPub->Publish(this->wrenchMsg);

  return true;
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive()
{
  return Sensor::IsActive() || this->wrenchPub->HasConnections();
}
