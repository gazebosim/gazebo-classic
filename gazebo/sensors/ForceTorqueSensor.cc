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
/* Desc: ForceTorque sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#include <sstream>

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/physics/Physics.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("force_torque", ForceTorqueSensor)

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
: Sensor()
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  // Create a publisher for the force_torque information.
  if (this->sdf->HasElement("force_torque") &&
      this->sdf->GetElement("force_torque")->HasElement("topic") &&
      this->sdf->GetElement("force_torque")->GetValueString("topic")
      != "__default_topic__")
  {
    // This will create a topic based on the name specified in SDF.
    this->forceTorquesPub = this->node->Advertise<msgs::JointWrench>(
      this->sdf->GetElement("force_torque")->GetValueString("topic"));
  }
  else
  {
    // This will create a topic based on the name of the parent and the
    // name of the sensor.
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName();
    boost::replace_all(topicName, "::", "/");

    this->forceTorquesPub = this->node->Advertise<msgs::JointWrench>(topicName);
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  if (!this->forceTorqueSub)
  {
    this->forceTorqueSub = this->node->Subscribe("~/physics/force_torque",
        &ForceTorqueSensor::OnForceTorques, this);
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Don't do anything if there is no new data to process.
  if (this->incomingForceTorques.size() == 0)
    return;

  // Clear the outgoing force_torque message.
  this->forceTorquesMsg.clear_wrench();

  // Iterate over all the force_torque messages
  for (ForceTorqueMsgs_L::iterator iter = this->incomingForceTorques.begin();
      iter != this->incomingForceTorques.end(); ++iter)
  {
    for (int i = 0; i < (*iter)->wrench_size(); ++i)
    {
      // Copy the force_torque message.
      msgs::JointWrench *forceTorqueMsg = this->forceTorquesMsg.add_wrench();
      forceTorqueMsg->CopyFrom((*iter)->wrench(i));
    }
  }

  // Clear the incoming force_torque list.
  this->incomingForceTorques.clear();

  this->lastMeasurementTime = this->world->GetSimTime();

  // Generate a outgoing message only if someone is listening.
  if (this->forceTorquesPub && this->forceTorquesPub->HasConnections())
  {
    msgs::Set(this->forceTorquesMsg.mutable_time(), this->lastMeasurementTime);
    this->forceTorquesPub->Publish(this->forceTorquesMsg);
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
void ForceTorqueSensor::OnForceTorques(ConstForceTorquePtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only store information if the sensor is active
  if (this->IsActive())
  {
    // Store the force_torque message for processing in UpdateImpl
    this->incomingForceTorques.push_back(_msg);

    // Prevent the incomingForceTorques list to grow indefinitely.
    if (this->incomingForceTorques.size() > 100)
      this->incomingForceTorques.pop_front();
  }
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive()
{
  return this->active ||
         (this->forceTorquesPub && this->forceTorquesPub->HasConnections());
}
