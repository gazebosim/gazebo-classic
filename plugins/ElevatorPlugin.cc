/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include "plugins/ElevatorPluginPrivate.hh"
#include "plugins/ElevatorPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

/////////////////////////////////////////////////
ElevatorPlugin::ElevatorPlugin()
  : dataPtr(new ElevatorPluginPrivate)
{
  this->dataPtr->doorController = NULL;
  this->dataPtr->liftController = NULL;
  this->dataPtr->doorWaitTime = common::Time(5, 0);
}

/////////////////////////////////////////////////
ElevatorPlugin::~ElevatorPlugin()
{
  this->dataPtr->updateConnection.reset();

  delete this->dataPtr->doorController;
  this->dataPtr->doorController = NULL;

  delete this->dataPtr->liftController;
  this->dataPtr->liftController = NULL;

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ElevatorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ElevatorPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "ElevatorPlugin sdf pointer is NULL");
  this->dataPtr->model = _model;
  this->dataPtr->sdf = _sdf;

  // Get the time to hold the door open.
  if (this->dataPtr->sdf->HasElement("door_wait_time"))
  {
    this->dataPtr->doorWaitTime.Set(
      this->dataPtr->sdf->Get<double>("door_wait_time"));
  }

  // Get the elevator topic.
  std::string elevatorTopic = "~/elevator";
  if (this->dataPtr->sdf->HasElement("topic"))
    elevatorTopic = this->dataPtr->sdf->Get<std::string>("topic");

  float floorHeight = 3.0;
  if (this->dataPtr->sdf->HasElement("floor_height"))
    floorHeight = this->dataPtr->sdf->Get<float>("floor_height");
  else
  {
    gzwarn << "No <floor_height> specified for elevator plugin. "
           << "Using a height of 3 meters. This value may cause "
           << "the elevator to move incorrectly.\n";
  }

  // Get the lift joint
  std::string liftJointName =
    this->dataPtr->sdf->Get<std::string>("lift_joint");

  this->dataPtr->liftJoint = this->dataPtr->model->GetJoint(liftJointName);
  if (!this->dataPtr->liftJoint)
  {
    gzerr << "Unable to find lift joint[" << liftJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Get the door joint
  std::string doorJointName =
    this->dataPtr->sdf->Get<std::string>("door_joint");

  this->dataPtr->doorJoint = this->dataPtr->model->GetJoint(doorJointName);
  if (!this->dataPtr->doorJoint)
  {
    gzerr << "Unable to find door joint[" << doorJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Create the door and lift controllers.
  this->dataPtr->doorController = new ElevatorPluginPrivate::DoorController(
      this->dataPtr->doorJoint);
  this->dataPtr->liftController = new ElevatorPluginPrivate::LiftController(
      this->dataPtr->liftJoint, floorHeight);

  // Connect to the update event.
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ElevatorPlugin::Update, this, _1));

  // Create the node for communication
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->GetName());

  // Subscribe to the elevator topic.
  this->dataPtr->elevatorSub = this->dataPtr->node->Subscribe(elevatorTopic,
      &ElevatorPlugin::OnElevator, this);
}

/////////////////////////////////////////////////
void ElevatorPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  // Currently we only expect the message to contain a floor to move to.
  try
  {
    this->MoveToFloor(std::stoi(_msg->data()));
  }
  catch(...)
  {
    gzerr << "Unable to process elevator message["
          << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void ElevatorPlugin::MoveToFloor(const int _floor)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Ignore messages when the elevator is currently busy.
  if (!this->dataPtr->states.empty())
    return;

  // Step 1: close the door.
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::CloseState(
        this->dataPtr->doorController));

  // Step 2: Move to the correct floor.
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::MoveState(
        _floor, this->dataPtr->liftController));

  // Step 3: Open the door
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::OpenState(
        this->dataPtr->doorController));

  // Step 4: Wait
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::WaitState(
        this->dataPtr->doorWaitTime));

  // Step 5: Close the door
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::CloseState(
        this->dataPtr->doorController));
}

/////////////////////////////////////////////////
void ElevatorPlugin::Update(const common::UpdateInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);

  // Process the states
  if (!this->dataPtr->states.empty())
  {
    // Update the front state, and remove it if the state is done
    if (this->dataPtr->states.front()->Update())
    {
      delete this->dataPtr->states.front();
      this->dataPtr->states.pop_front();
    }
  }

  // Update the controllers
  this->dataPtr->doorController->Update(_info);
  this->dataPtr->liftController->Update(_info);
}

////////////////////////////////////////////////
void ElevatorPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  for (auto s: this->dataPtr->states)
    delete s;
  this->dataPtr->states.clear();
  this->dataPtr->doorController->Reset();
  this->dataPtr->liftController->Reset();
}

////////////////////////////////////////////////
// ElevatorPluginPrivate Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::~ElevatorPluginPrivate()
{
  delete this->doorController;
  this->doorController = NULL;

  delete this->liftController;
  this->liftController = NULL;

  for (auto s: this->states)
    delete s;
  this->states.clear();
}

////////////////////////////////////////////////
// CloseState Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::CloseState::CloseState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::CloseState::Start()
{
  this->ctrl->SetTarget(ElevatorPluginPrivate::DoorController::CLOSE);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::CloseState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() ==
      ElevatorPluginPrivate::DoorController::CLOSE &&
      this->ctrl->GetState() ==
      ElevatorPluginPrivate::DoorController::STATIONARY;
  }
}

////////////////////////////////////////////////
// OpenState Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::OpenState::OpenState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::OpenState::Start()
{
  this->ctrl->SetTarget(ElevatorPluginPrivate::DoorController::OPEN);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::OpenState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() ==
      ElevatorPluginPrivate::DoorController::OPEN &&
      this->ctrl->GetState() ==
      ElevatorPluginPrivate::DoorController::STATIONARY;
  }
}

////////////////////////////////////////////////
// MoveState Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::MoveState::MoveState(int _floor, LiftController *_ctrl)
  : State(), floor(_floor), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::MoveState::Start()
{
  this->ctrl->SetFloor(this->floor);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::MoveState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetState() ==
      ElevatorPluginPrivate::LiftController::STATIONARY;
  }
}

////////////////////////////////////////////////
// WaitState Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::WaitState::WaitState(const common::Time &_waitTime)
  : State(), waitTimer(_waitTime, true)
{
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::WaitState::Start()
{
  this->waitTimer.Reset();
  this->waitTimer.Start();
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::WaitState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    if (this->waitTimer.GetElapsed() == common::Time::Zero)
      return true;
    else
      return false;
  }
}

////////////////////////////////////////////////
// DoorController Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::DoorController::DoorController(
    physics::JointPtr _doorJoint)
  : doorJoint(_doorJoint), state(STATIONARY), target(CLOSE)
{
  this->doorPID.Init(2, 0, 1.0);
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::DoorController::SetTarget(
    ElevatorPluginPrivate::DoorController::Target _target)
{
  this->target = _target;
}

/////////////////////////////////////////////////
ElevatorPluginPrivate::DoorController::Target
ElevatorPluginPrivate::DoorController::GetTarget() const
{
  return this->target;
}

/////////////////////////////////////////////////
ElevatorPluginPrivate::DoorController::State
ElevatorPluginPrivate::DoorController::GetState() const
{
  return this->state;
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::DoorController::Reset()
{
  this->prevSimTime = common::Time::Zero;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::DoorController::Update(
    const common::UpdateInfo &_info)
{
  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return false;
  }

  double errorTarget = this->target == OPEN ? 1.0 : 0.0;

  double doorError = this->doorJoint->GetAngle(0).Radian() -
    errorTarget;

  double doorForce = this->doorPID.Update(doorError,
      _info.simTime - this->prevSimTime);

  this->doorJoint->SetForce(0, doorForce);

  if (std::abs(doorError) < 0.05)
  {
    this->state = STATIONARY;
    return true;
  }
  else
  {
    this->state = MOVING;
    return false;
  }
}

////////////////////////////////////////////////
// LiftController Class

/////////////////////////////////////////////////
ElevatorPluginPrivate::LiftController::LiftController(
    physics::JointPtr _liftJoint, float _floorHeight)
  : state(STATIONARY), floor(0), floorHeight(_floorHeight),
    liftJoint(_liftJoint)
{
  this->liftPID.Init(100000, 0, 100000.0);
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::LiftController::Reset()
{
  this->prevSimTime = common::Time::Zero;
}

/////////////////////////////////////////////////
bool ElevatorPluginPrivate::LiftController::Update(
    const common::UpdateInfo &_info)
{
  // Bootstrap the time.
  if (this->prevSimTime == common::Time::Zero)
  {
    this->prevSimTime = _info.simTime;
    return false;
  }

  double error = this->liftJoint->GetAngle(0).Radian() -
    (this->floor * this->floorHeight);

  double force = this->liftPID.Update(error, _info.simTime - this->prevSimTime);
  this->prevSimTime = _info.simTime;

  this->liftJoint->SetForce(0, force);

  if (std::abs(error) < 0.15)
  {
    this->state = ElevatorPluginPrivate::LiftController::STATIONARY;
    return true;
  }
  else
  {
    this->state = ElevatorPluginPrivate::LiftController::MOVING;
    return false;
  }
}

/////////////////////////////////////////////////
void ElevatorPluginPrivate::LiftController::SetFloor(int _floor)
{
  this->floor = _floor;
}

/////////////////////////////////////////////////
int ElevatorPluginPrivate::LiftController::GetFloor() const
{
  return this->floor;
}

/////////////////////////////////////////////////
ElevatorPluginPrivate::LiftController::State
ElevatorPluginPrivate::LiftController::GetState() const
{
  return this->state;
}


