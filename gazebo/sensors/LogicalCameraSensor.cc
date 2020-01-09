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
: Sensor(sensors::OTHER),
  dataPtr(new LogicalCameraSensorPrivate)
{
}

//////////////////////////////////////////////////
LogicalCameraSensor::~LogicalCameraSensor()
{
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Load(const std::string &_worldName,
                               sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  // Get a pointer to the parent link. This will be used to adjust the
  // orientation of the logical camera.
  physics::EntityPtr parentEntity =
    this->world->EntityByName(this->ParentName());
  this->dataPtr->parentLink =
    boost::dynamic_pointer_cast<physics::Link>(parentEntity);

  // Store parent model's name for use in the UpdateImpl function.
  this->dataPtr->modelName =
    this->dataPtr->parentLink->GetModel()->GetScopedName();
}

//////////////////////////////////////////////////
std::string LogicalCameraSensor::Topic() const
{
  std::string topicName = Sensor::Topic();
  if (topicName.empty())
  {
    topicName = "~/" + this->ParentName() + "/" + this->Name() + "/models";
    boost::replace_all(topicName, "::", "/");
  }

  return topicName;
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  // Create publisher of the logical camera images
  this->dataPtr->pub =
    this->node->Advertise<msgs::LogicalCameraImage>(this->Topic(), 50);
}

//////////////////////////////////////////////////
void LogicalCameraSensor::Init()
{
  // Read configuration values
  if (this->sdf->HasElement("logical_camera"))
  {
    sdf::ElementPtr cameraSdf =
      this->sdf->GetElement("logical_camera");

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
void LogicalCameraSensorPrivate::AddVisibleModels(
    ignition::math::Pose3d &_myPose, const physics::Model_V &_models)
{
  for (auto const &model : _models)
  {
    auto const &scopedName = model->GetScopedName();
    auto const aabb = model->BoundingBox();

    if (this->modelName != scopedName && this->frustum.Contains(aabb))
    {
      // Add new model msg
      msgs::LogicalCameraImage::Model *modelMsg = this->msg.add_model();

      // Set the name and pose reported by the sensor.
      modelMsg->set_name(scopedName);
      msgs::Set(modelMsg->mutable_pose(),
          model->WorldPose() - _myPose);
    }
    // Check nested models
    // Note, the model AABB does not necessarily contain the nested model
    // so nested models must be searched even if the frustum does not contain
    // the parent model.
    AddVisibleModels(_myPose, model->NestedModels());
  }
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::UpdateImpl(const bool _force)
{
  // Only compute if active, or the update is forced
  if (_force || this->IsActive())
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->msg.clear_model();

    // Get the pose of the camera's parent.
    ignition::math::Pose3d myPose = this->pose +
      this->dataPtr->parentLink->WorldPose();

    // Update the pose of the frustum.
    this->dataPtr->frustum.SetPose(myPose);

    // Set the camera's pose in the message.
    msgs::Set(this->dataPtr->msg.mutable_pose(), myPose);

    // Recursively check if models and nested models are in the frustum.
    this->dataPtr->AddVisibleModels(myPose, this->world->Models());

    // Send the message.
    this->dataPtr->pub->Publish(this->dataPtr->msg);
  }

  return true;
}

//////////////////////////////////////////////////
bool LogicalCameraSensor::IsActive() const
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
