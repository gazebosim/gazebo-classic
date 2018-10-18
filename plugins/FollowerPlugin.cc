/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>
#include <mutex>
#include <string>

#include <ignition/math/Box.hh>

#include <sdf/sdf.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "plugins/FollowerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FollowerPlugin)

namespace gazebo
{
  /// \internal
  /// \brief Private data for the FollowerPlugin class
  struct FollowerPluginPrivate
  {
    /// \brief Pointer to the update event connection.
    public: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model.
    public: physics::ModelPtr model;

    /// \brief Update mutex.
    public: std::mutex mutex;

    /// \brief Local copy of input image.
    public: msgs::Image imageMsg;

    /// \brief Revolute joint for moving the left wheel of the vehicle.
    public: physics::JointPtr leftJoint;

    /// \brief Revolute joint for moving the right wheel of the vehicle.
    public: physics::JointPtr rightJoint;

    /// \brief Left/Right wheel speed.
    public: double wheelSpeed[2];

    /// \brief Wheel separation.
    public: double wheelSeparation;

    /// \brief Wheel radius.
    public: double wheelRadius;

    /// \brief Connection to the depth camera frame event.
    public: event::ConnectionPtr newDepthFrameConnection;

    /// \brief Pointer to the depth camera
    public: rendering::DepthCameraPtr depthCamera;

    /// \brief Depth image data buffer
    public: float *depthBuffer = NULL;
  };
}

// Used for left/right wheel.
enum {RIGHT, LEFT};

/////////////////////////////////////////////////
FollowerPlugin::FollowerPlugin()
  : dataPtr(new FollowerPluginPrivate)
{
  this->dataPtr->wheelSpeed[LEFT] = this->dataPtr->wheelSpeed[RIGHT] = 0;
  this->dataPtr->wheelSeparation = 1.0;
  this->dataPtr->wheelRadius = 1.0;
}

/////////////////////////////////////////////////
FollowerPlugin::~FollowerPlugin()
{
  this->dataPtr->newDepthFrameConnection.reset();
  this->dataPtr->updateConnection.reset();
  if (this->dataPtr->depthBuffer != NULL)
    delete [] this->dataPtr->depthBuffer;
}

/////////////////////////////////////////////////
void FollowerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FollowerPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FollowerPlugin _sdf pointer is NULL");

  if (_model == NULL || _sdf == NULL)
  {
    gzerr << "Failed to load FollowerPlugin. NULL model or sdf" << std::endl;
    return;
  }

  this->dataPtr->model = _model;

  // find depth camera sensor
  if (!this->FindSensor(this->dataPtr->model))
  {
    gzerr << "depth sensor not found!" << std::endl;
    return;
  }

  // diff drive params
  if (_sdf->HasElement("left_joint"))
  {
    this->dataPtr->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  }

  if (_sdf->HasElement("right_joint"))
  {
    this->dataPtr->rightJoint = _model->GetJoint(
        _sdf->GetElement("right_joint")->Get<std::string>());
  }

  if (!this->dataPtr->leftJoint || !this->dataPtr->rightJoint)
    this->FindJoints();

  if (!this->dataPtr->leftJoint || !this->dataPtr->rightJoint)
  {
    gzerr << "left or right joint not found!" << std::endl;
    return;
  }

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&FollowerPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void FollowerPlugin::Init()
{
  if (!this->dataPtr->leftJoint || !this->dataPtr->rightJoint)
    return;

  this->dataPtr->wheelSeparation =
      this->dataPtr->leftJoint->GetAnchor(0).Distance(
      this->dataPtr->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->dataPtr->leftJoint->GetChild());

  ignition::math::Box bb = parent->GetBoundingBox().Ign();
  // This assumes that the largest dimension of the wheel is the diameter
  this->dataPtr->wheelRadius = bb.Size().Max() * 0.5;
}

/////////////////////////////////////////////////
void FollowerPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->imageMsg.Clear();
}

/////////////////////////////////////////////////
void FollowerPlugin::FindJoints()
{
  // assumes the first two revolute joints are the ones connecting the
  // wheels to the chassis
  auto joints = this->dataPtr->model->GetJoints();
  if (joints.size() < 2u)
    return;

  physics::Joint_V revJoints;
  for (const auto &j : joints)
  {
    if (j->GetMsgType() == msgs::Joint::REVOLUTE)
      revJoints.push_back(j);
  }

  if (revJoints.size() < 2u)
    return;

  this->dataPtr->leftJoint = revJoints[0];
  this->dataPtr->rightJoint = revJoints[1];
}

/////////////////////////////////////////////////
bool FollowerPlugin::FindSensor(const physics::ModelPtr &_model)
{
  // loop through links to find depth sensor
  for (const auto l : _model->GetLinks())
  {
    for (unsigned int i = 0; i < l->GetSensorCount(); ++i)
    {
      std::string sensorName = l->GetSensorName(i);
      sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
      if (!sensor)
        continue;

      if (sensor->Type() == "depth")
      {
        sensors::DepthCameraSensorPtr depthSensor =
            std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
        if (depthSensor)
        {
          rendering::DepthCameraPtr camera =
              depthSensor->DepthCamera();
          if (camera)
          {
            this->dataPtr->depthCamera = camera;
            this->dataPtr->newDepthFrameConnection =
                this->dataPtr->depthCamera->ConnectNewDepthFrame(
                std::bind(&FollowerPlugin::OnNewDepthFrame, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4,
                std::placeholders::_5));
            return true;
          }
        }
      }
    }
  }

  // recursively look for sensor in nested models
  for (const auto &m : _model->NestedModels())
  {
    if (this->FindSensor(m))
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
void FollowerPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update follower.
  this->UpdateFollower();
}

/////////////////////////////////////////////////
void FollowerPlugin::OnNewDepthFrame(const float *_image,
    const unsigned int _width, const unsigned int _height,
    const unsigned int /*_depth*/, const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  float f;
  unsigned int depthBufferSize = _width*_height*sizeof(f);
  if (_width != this->dataPtr->imageMsg.width() ||
      _height != this->dataPtr->imageMsg.height())
  {
    if (this->dataPtr->depthBuffer != NULL)
      delete [] this->dataPtr->depthBuffer;
    this->dataPtr->depthBuffer = new float[depthBufferSize];
    this->dataPtr->imageMsg.set_width(_width);
    this->dataPtr->imageMsg.set_height(_height);
  }

  memcpy(this->dataPtr->depthBuffer, _image, depthBufferSize);
}

/////////////////////////////////////////////////
void FollowerPlugin::UpdateFollower()
{
  if (this->dataPtr->imageMsg.width() == 0u ||
      this->dataPtr->imageMsg.height() == 0u)
  {
    return;
  }

  double minRange = 0.1;
  double maxRange = 5;

  // Find closest point.
  int mid = this->dataPtr->imageMsg.height() * 0.5;
  float minDepth = maxRange + 1;
  int idx = -1;
  for (unsigned int i = 0; i < this->dataPtr->imageMsg.width(); ++i)
  {
    float d =
        this->dataPtr->depthBuffer[mid * this->dataPtr->imageMsg.width() + i];
    if (d > minRange && d < maxRange && d < minDepth)
    {
      // Update minimum depth.
      minDepth = d;
      // Store index of pixel with min range.
      idx = i;
    }
  }

  // brake if too close
  if (idx < 0 || minDepth < 0.4)
  {
    // Brakes on!
    this->dataPtr->leftJoint->SetVelocity(0, 0);
    this->dataPtr->rightJoint->SetVelocity(0, 0);
    return;
  }

  // Set turn rate based on idx of min range in the image.
  double turn = -(idx / (this->dataPtr->imageMsg.width() / 2.0)) + 1.0;

  double vr = -0.1;
  double maxTurnRate = 0.1;

  double va = turn * maxTurnRate;

  this->dataPtr->wheelSpeed[LEFT] =
      vr + va * this->dataPtr->wheelSeparation / 2.0;
  this->dataPtr->wheelSpeed[RIGHT] =
      vr - va * this->dataPtr->wheelSeparation / 2.0;

  double leftVelDesired =
      (this->dataPtr->wheelSpeed[LEFT] / this->dataPtr->wheelRadius);
  double rightVelDesired =
      (this->dataPtr->wheelSpeed[RIGHT] / this->dataPtr->wheelRadius);

  this->dataPtr->leftJoint->SetVelocity(0, leftVelDesired);
  this->dataPtr->rightJoint->SetVelocity(0, rightVelDesired);
}
