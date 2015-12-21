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

#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/FollowerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FollowerPlugin)

// Used for left/right wheel.
enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
FollowerPlugin::FollowerPlugin()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
  this->wheelSeparation = 1.0;
  this->wheelRadius = 1.0;
}

/////////////////////////////////////////////////
FollowerPlugin::~FollowerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
}

/////////////////////////////////////////////////
void FollowerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FollowerPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FollowerPlugin _sdf pointer is NULL");
  this->model = _model;

  // Read the required parameters
  if (_sdf->HasElement("depth_image_topic"))
    this->imageTopic = _sdf->Get<std::string>("depth_image_topic");

  if (this->imageTopic.empty())
    this->FindSensor(this->model);

  // diff drive params
  if (_sdf->HasElement("left_joint"))
  {
    this->leftJoint = _model->GetJoint(
      _sdf->GetElement("left_joint")->Get<std::string>());
  }

  if (_sdf->HasElement("right_joint"))
  {
    this->rightJoint = _model->GetJoint(
        _sdf->GetElement("right_joint")->Get<std::string>());
  }

  if (!this->leftJoint || !this->rightJoint)
    this->FindJoints();

  if (!this->leftJoint || !this->rightJoint)
  {
    gzerr << "left or right joint not found!" << std::endl;
    return;
  }

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&FollowerPlugin::Update, this, _1));

  // Initialize transport.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->imageSub = this->node->Subscribe(this->imageTopic,
      &FollowerPlugin::OnImage, this);
}

/////////////////////////////////////////////////
void FollowerPlugin::Init()
{
  if (!this->leftJoint || !this->rightJoint)
    return;

  this->wheelSeparation = this->leftJoint->GetAnchor(0).Distance(
      this->rightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->leftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;
}

/////////////////////////////////////////////////
void FollowerPlugin::FindJoints()
{
  auto joints = this->model->GetJoints();
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

  this->leftJoint = revJoints[0];
  this->rightJoint = revJoints[1];
}

/////////////////////////////////////////////////
bool FollowerPlugin::FindSensor(physics::ModelPtr _model)
{
  sensors::SensorPtr depthSensor;
  for (const auto l : _model->GetLinks())
  {
    for (unsigned int i = 0; i < l->GetSensorCount(); ++i)
    {
      std::string sensorName = l->GetSensorName(i);
      sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
      if (!sensor)
        continue;

      if (sensor->GetType() == "depth")
      {
        depthSensor = sensor;
        this->imageTopic = depthSensor->GetTopic();
        return true;
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
void FollowerPlugin::Update(const common::UpdateInfo &/*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Update follower.
  this->UpdateFollower();
}

/////////////////////////////////////////////////
void FollowerPlugin::OnImage(ConstImageStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->imageMsg.CopyFrom(_msg->image());
}

/////////////////////////////////////////////////
void FollowerPlugin::UpdateFollower()
{
  double minRange = 0.1;
  double maxRange = 5;

  // Find closest point.
  int mid = this->imageMsg.height() * 0.5;

  unsigned int depthSamples = this->imageMsg.width() * this->imageMsg.height();
  float f;
  // cppchecker recommends using sizeof(varname)
  unsigned int depthBufferSize = depthSamples * sizeof(f);
  float *depthBuffer = new float[depthSamples];
  memcpy(depthBuffer, this->imageMsg.data().c_str(), depthBufferSize);

  float minDepth = maxRange + 1;
  int idx = -1;
  for (unsigned int i = 0; i < this->imageMsg.width(); ++i)
  {
    float d = depthBuffer[mid * this->imageMsg.width() + i];
    if (d > minRange && d < maxRange && d < minDepth)
    {
      // Update minimum depth.
      minDepth = d;
      // Store index of pixel with min range.
      idx = i;
    }
  }
  delete[] depthBuffer;

  if (idx < 0 || minDepth < 0.4)
  {
    // Brakes on!
    this->leftJoint->SetVelocity(0, 0);
    this->rightJoint->SetVelocity(0, 0);
    return;
  }

  // Set turn rate based on idx of min range in the image.
  double turn = -(idx / (this->imageMsg.width() / 2.0)) + 1.0;

  double vr = -0.1;
  double maxTurnRate = 0.1;

  double va = turn * maxTurnRate;

  this->wheelSpeed[LEFT] = vr + va * this->wheelSeparation / 2.0;
  this->wheelSpeed[RIGHT] = vr - va * this->wheelSeparation / 2.0;

  double leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius);
  double rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);

  this->leftJoint->SetVelocity(0, leftVelDesired);
  this->rightJoint->SetVelocity(0, rightVelDesired);
}
