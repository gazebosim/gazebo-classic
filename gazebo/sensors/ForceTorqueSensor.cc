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
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  GZ_ASSERT(this->world != NULL,
      "SonarSensor did not get a valid World pointer");

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
  return msgs::Convert(this->wrenchMsg.wrench().force());
}

//////////////////////////////////////////////////
math::Vector3 ForceTorqueSensor::GetTorque() const
{
  return msgs::Convert(this->wrenchMsg.wrench().torque());
}

//////////////////////////////////////////////////
void ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->wrenchMsg.mutable_time(), this->lastMeasurementTime);

  physics::JointWrench wrench = this->parentJoint->GetForceTorque(0u);

  // Get the force and torque in the parent frame.
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_force(),
      wrench.body2Force);
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_torque(),
      wrench.body2Torque);

  this->update(this->wrenchMsg);

  if (this->wrenchPub)
    this->wrenchPub->Publish(this->wrenchMsg);
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive()
{
  return Sensor::IsActive() || this->wrenchPub->HasConnections();
}
