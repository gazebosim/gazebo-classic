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
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "Small2dGimbalPlugin.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(Small2dGimbalPlugin)

/// \brief Private data class
class gazebo::Small2dGimbalPluginPrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void OnStringMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr sub;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for tilting the gimbal
  public: physics::JointPtr tiltJoint;

  /// \brief Command that updates the gimbal tilt angle
  public: double command = IGN_PI_2;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief PID controller for the gimbal
  public: common::PID pid;

  /// \brief Last update sim time
  public: common::Time lastUpdateTime;
};

/////////////////////////////////////////////////
Small2dGimbalPlugin::Small2dGimbalPlugin()
  : dataPtr(new Small2dGimbalPluginPrivate)
{
  this->dataPtr->pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

/////////////////////////////////////////////////
void Small2dGimbalPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->model = _model;
  std::string jointName = _model->GetName() + "::gimbal_small_2d::tilt_joint";
  this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(jointName);

  if (!this->dataPtr->tiltJoint)
  {
    gzerr << "Small2dGimbalPlugin::Load ERROR! Can't get joint '"
          << jointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void Small2dGimbalPlugin::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->GetName());

  this->dataPtr->lastUpdateTime =
    this->dataPtr->model->GetWorld()->GetSimTime();

  std::string topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_tilt_cmd";
  this->dataPtr->sub = this->dataPtr->node->Subscribe(topic,
      &Small2dGimbalPluginPrivate::OnStringMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Small2dGimbalPlugin::OnUpdate, this)));

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_tilt_status";

  this->dataPtr->pub =
    this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);
}

/////////////////////////////////////////////////
void Small2dGimbalPluginPrivate::OnStringMsg(ConstGzStringPtr &_msg)
{
  this->command = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void Small2dGimbalPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint)
    return;

  double angle = this->dataPtr->tiltJoint->GetAngle(0).Radian();

  common::Time time = this->dataPtr->model->GetWorld()->GetSimTime();
  if (time < this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  else if (time > this->dataPtr->lastUpdateTime)
  {
    double dt = (this->dataPtr->lastUpdateTime - time).Double();
    double error = angle - this->dataPtr->command;
    double force = this->dataPtr->pid.Update(error, dt);
    this->dataPtr->tiltJoint->SetForce(0, force);
    this->dataPtr->lastUpdateTime = time;
  }

  static int i = 1000;
  if (++i > 100)
  {
    i = 0;
    std::stringstream ss;
    ss << angle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->dataPtr->pub->Publish(m);
  }
}
