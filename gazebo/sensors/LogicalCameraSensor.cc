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

#include <boost/algorithm/string.hpp>
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

  // Get a pointer to the parent link. This will be used to adjust the
  // orientation of the logical camera.
  physics::EntityPtr parentEntity = this->world->GetEntity(this->parentName);
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  // Store parent model's name for use in the UpdateImpl function.
  this->dataPtr->modelName = this->dataPtr->parentLink->GetModel()->GetName();
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

  // Create publisher of the logical camera images
  this->dataPtr->pub = this->node->Advertise<msgs::LogicalCameraImage>(
      this->GetTopic(), 50);
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Init()
{
  // Read configuration values
  if (this->sdf->HasElement("logical_camera"))
  {
    sdf::ElementPtr cameraSdf = this->sdf->GetElement("logical_camera");

    // These values are required in SDF, so no need to check for their
    // existence.
    this->dataPtr->frustum.SetNear(cameraSdf->Get<double>("near"));
    this->dataPtr->frustum.SetFar(cameraSdf->Get<double>("far"));
    this->dataPtr->frustum.SetFOV(cameraSdf->Get<double>("horizontal_fov"));
    this->dataPtr->frustum.SetAspectRatio(
        cameraSdf->Get<double>("aspect_ratio"));
  }

  Sensor::Init();
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::UpdateImpl(bool _force)
{
  // Only compute if active, or the update is forced
  if (_force || this->IsActive())
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->msg.clear_model();

    // Get the pose of the camera's parent.
    ignition::math::Pose3d myPose = this->pose +
      this->dataPtr->parentLink->GetWorldPose().Ign();

    // Update the pose of the frustum.
    this->dataPtr->frustum.SetPose(myPose);

    // Set the camera's pose in the message.
    msgs::Set(this->dataPtr->msg.mutable_pose(), myPose);

    // Check all models for inclusion in the frustum.
    for (auto const &model : this->world->GetModels())
    {
      // Add the the model to the output if it is in the frustum, and
      // we are not detecting ourselves.
      if (this->dataPtr->modelName != model->GetName() &&
          this->dataPtr->frustum.Contains(model->GetBoundingBox().Ign()))
      {
        // Add new model msg
        msgs::LogicalCameraImage::Model *modelMsg =
          this->dataPtr->msg.add_model();

        // Set the name and pose reported by the sensor.
        modelMsg->set_name(model->GetScopedName());
        msgs::Set(modelMsg->mutable_pose(),
            model->GetWorldPose().Ign() - myPose);
      }
    }

    // Send the message.
    this->dataPtr->pub->Publish(this->dataPtr->msg);
  }

  return true;
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::IsActive()
{
  return Sensor::IsActive() ||
    (this->dataPtr->pub && this->dataPtr->pub->HasConnections());
}

//////////////////////////////////////////////////
double LogicalCameraSensor::Near() const
{
  return this->dataPtr->frustum.Near();
}

//////////////////////////////////////////////////
double LogicalCameraSensor::Far() const
{
  return this->dataPtr->frustum.Far();
}

//////////////////////////////////////////////////
ignition::math::Angle LogicalCameraSensor::HorizontalFOV() const
{
  return this->dataPtr->frustum.FOV();
}

//////////////////////////////////////////////////
double LogicalCameraSensor::AspectRatio() const
{
  return this->dataPtr->frustum.AspectRatio();
}

//////////////////////////////////////////////////
msgs::LogicalCameraImage LogicalCameraSensor::Image() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->msg;
}
