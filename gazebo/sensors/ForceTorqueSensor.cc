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
/* Desc: Force Torque sensor
 * Author: John Hsu
 * Date: 6 September 2008
*/


#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("force_torque", ForceTorqueSensor)

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
  : Sensor(sensors::OTHER)
{
  this->dataIndex = 0;
  this->dataDirty = false;
  this->incomingLinkData[0].reset();
  this->incomingLinkData[1].reset();
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  if (this->sdf->HasElement("force_torque") &&
      this->sdf->GetElement("force_torque")->HasElement("topic") &&
      this->sdf->GetElement("force_torque")->GetValueString("topic")
      != "__default_topic__")
  {
    this->pub = this->node->Advertise<msgs::ForceTorque>(
        this->sdf->GetElement("force_torque")->GetValueString("topic"));
  }
  else
  {
    std::string topicName = "~/";
    topicName += this->parentName + "/" + this->GetName() + "/force_torque";
    boost::replace_all(topicName, "::", "/");

    this->pub = this->node->Advertise<msgs::ForceTorque>(topicName);
  }

  // Handle noise model settings.
  this->noiseActive = false;
  sdf::ElementPtr forceTorqueElem = this->sdf->GetElement("force_torque");
  if (forceTorqueElem->HasElement("noise"))
  {
    sdf::ElementPtr noiseElem = forceTorqueElem->GetElement("noise");
    std::string type = noiseElem->GetValueString("type");
    if (type == "gaussian")
    {
      this->noiseActive = true;
      this->noiseType = GAUSSIAN;
      this->noiseMean = 0.0;
      this->noiseStdDev = 0.0;
      this->bias = 0.0;
      this->noiseMean = noiseElem->GetValueDouble("mean");
      this->noiseStdDev = noiseElem->GetValueDouble("stddev");
      double accelBiasMean = noiseElem->GetValueDouble("bias_mean");
      double accelBiasStddev = noiseElem->GetValueDouble("bias_stddev");
      // Sample the bias that we'll use later
      this->bias = math::Rand::GetDblNormal(accelBiasMean,
                                                 accelBiasStddev);
      // With equal probability, we pick a negative bias (by convention,
      // accelBiasMean should be positive, though it would work fine if
      // negative).
      if (math::Rand::GetDblUniform() < 0.5)
        this->bias = -this->bias;
      gzlog << "applying Gaussian noise model to accel with mean " <<
        this->noiseMean << " and stddev " << this->noiseStdDev <<
        ", bias " << this->bias << std::endl;
    }
    else
      gzwarn << "ignoring unknown noise model type \"" << type << "\"" <<
        std::endl;
  }

  this->parentEntity->SetPublishData(true);

  std::string topic = "~/" + this->parentEntity->GetScopedName();
  this->linkDataSub = this->node->Subscribe(topic,
    &ForceTorqueSensor::OnLinkData, this);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->parentEntity = boost::dynamic_pointer_cast<physics::Link>(
      this->world->GetEntity(this->parentName));

  if (!this->parentEntity)
  {
    gzthrow("ForceTorque has invalid parent[" + this->parentName +
            "]. Must be a link\n");
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Fini()
{
  this->parentEntity->SetPublishData(false);
  this->pub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
msgs::ForceTorque ForceTorqueSensor::GetForceTorqueMessage() const
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->forceTorqueMsg;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::OnLinkData(ConstLinkDataPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  // Store the contacts message for processing in UpdateImpl
  this->incomingLinkData[this->dataIndex] = _msg;
  this->dataDirty = true;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  msgs::LinkData msg;
  int readIndex = 0;

  {
    boost::mutex::scoped_lock lock(this->mutex);

    // Don't do anything if there is no new data to process.
    if (!this->dataDirty)
      return;

    readIndex = this->dataIndex;
    this->dataIndex ^= 1;
    this->dataDirty = false;
  }

  // toggle the index
  msg.CopyFrom(*this->incomingLinkData[readIndex].get());

  common::Time timestamp = msgs::Convert(msg.time());

  double dt = (timestamp - this->lastMeasurementTime).Double();

  if (dt > 0.0)
  {
    boost::mutex::scoped_lock lock(this->mutex);

    /// \TODO: fill message

    if (this->noiseActive)
    {
      switch (this->noiseType)
      {
        case GAUSSIAN:
          // Add Gaussian noise + fixed bias to each data stream
          this->forceTorqueMsg.mutable_wrench()->mutable_body_2_force()->set_x(
            this->forceTorqueMsg.wrench()->body_2_force().x() + this->bias +
            math::Rand::GetDblNormal(this->noiseMean,
                                     this->noiseStdDev));
          this->forceTorqueMsg.mutable_wrench()->mutable_body_2_force()->set_y(
            this->forceTorqueMsg.wrench()->body_2_force().y() + this->bias +
            math::Rand::GetDblNormal(this->noiseMean,
                                     this->noiseStdDev));
          this->forceTorqueMsg.mutable_wrench()->mutable_body_2_force()->set_z(
            this->forceTorqueMsg.wrench()->body_2_force().z() + this->bias +
            math::Rand::GetDblNormal(this->noiseMean,
                                     this->noiseStdDev));
          break;
        default:
          GZ_ASSERT(false, "Invalid noise model type");
      }
    }

    if (this->pub)
      this->pub->Publish(this->forceTorqueMsg);
  }
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive()
{
  return this->active ||
         (this->pub && this->pub->HasConnections());
}
