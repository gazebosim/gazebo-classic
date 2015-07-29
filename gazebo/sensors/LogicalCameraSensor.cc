/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/LogicalCameraSensorPrivate.hh"
#include "gazebo/sensors/LogicalCameraSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("logical_camera", LogicalCameraSensor)

//////////////////////////////////////////////////
LogicalCameraSensor::LogicalCameraSensor()
  : Sensor(sensors::OTHER), dataPtr(new LogicalCameraSensorPrivate())
{
}

//////////////////////////////////////////////////
LogicalCameraSensor::~LogicalCameraSensor()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Load(const std::string &_worldName,
                               sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
std::string LogicalCameraSensor::GetTopic() const
{
  std::string topicName = "~/" + this->parentName + "/" + this->GetName() +
    "/models";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->dataPtr->pub = this->node->Advertise<msgs::LogicalCameraImage>(
      this->GetTopic(), 50);
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Init()
{
  std::string worldName = this->world->GetName();

  if (!worldName.empty())
  {
  //  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");

  //  math::Pose cameraPose = this->pose;
  //  if (cameraSdf->HasElement("pose"))
  //    cameraPose = cameraSdf->Get<math::Pose>("pose") + cameraPose;
  }
  else
    gzerr << "No world name\n";

  Sensor::Init();
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::UpdateImpl(bool /*_force*/)
{
  // Update the pose of the frustum
  this->dataPtr->frustum.SetPose(this->pose);

  // Only compute if there are connections
  if (this->dataPtr->pub && this->dataPtr->pub->HasConnections())
  {
    msgs::LogicalCameraImage msg;

    // Check all models for inclusion in the frustum
    for (auto const &model : this->world->GetModels())
    {
      // Add the the model the output if it is in the frustum.
      if (this->dataPtr->frustum.Contains(model->GetWorldPose().pos.Ign()))
      {
        // Add new model msg
        msgs::LogicalCameraImage::Model *modelMsg = msg.add_model();

        // Set the name and pose reported by the sensor.
        modelMsg->set_name(model->GetScopedName());
        msgs::Set(modelMsg->mutable_pose(), model->GetWorldPose().Ign());
      }
    }

    this->dataPtr->pub->Publish(msg);
  }
  return true;
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
}
