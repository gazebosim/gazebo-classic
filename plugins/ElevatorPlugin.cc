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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

#include "plugins/ElevatorPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

/////////////////////////////////////////////////
ElevatorPlugin::ElevatorPlugin()
  : doorController(NULL), liftController(NULL)
{
}

/////////////////////////////////////////////////
ElevatorPlugin::~ElevatorPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  delete this->doorController;
  this->doorController = NULL;

  delete this->liftController;
  this->liftController = NULL;
}

/////////////////////////////////////////////////
void ElevatorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ElevatorPlugin model pointer is NULL");
  GZ_ASSERT(_sdf, "ElevatorPlugin sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  // Get the elevator topic.
  std::string elevatorTopic = "~/elevator";
  if (this->sdf->HasElement("topic"))
    elevatorTopic = this->sdf->Get<std::string>("topic");

  if (this->sdf->HasElement("floor_height"))
    this->floorHeight = this->sdf->Get<float>("floor_height");
  else
  {
    this->floorHeight = 3.0;
    gzwarn << "No <floor_height> specified for elevator plugin. "
           << "Using a height of 3 meters. This value may cause "
           << "the elevator to move incorrectly.\n";
  }

  // Get the lift joint
  std::string liftJointName = this->sdf->Get<std::string>("lift_joint");
  this->liftJoint = this->model->GetJoint(liftJointName);
  if (!this->liftJoint)
  {
    gzerr << "Unable to find lift joint[" << liftJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Get the door joint
  std::string doorJointName = this->sdf->Get<std::string>("door_joint");
  this->doorJoint = this->model->GetJoint(doorJointName);
  if (!this->doorJoint)
  {
    gzerr << "Unable to find door joint[" << doorJointName << "].\n";
    gzerr << "The elevator plugin is disabled.\n";
    return;
  }

  // Create the door and lift controllers.
  this->doorController = new DoorController(this->doorJoint);
  this->liftController = new LiftController(this->liftJoint);

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ElevatorPlugin::Update, this));

  // Create the node for communication
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  // Subscribe to the elevator topic.
  this->elevatorSub = this->node->Subscribe(elevatorTopic,
      &ElevatorPlugin::OnElevator, this);
}

/////////////////////////////////////////////////
void ElevatorPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  std::cout << "OnElevator[" << _msg->DebugString() << "]\n";
  std::lock_guard<std::mutex> lock(this->stateMutex);

  // Ignore messages when the elevator is currently busy.
  if (!this->states.empty())
    return;

  // Currently we only expect the message to contain a floor to move to.
  try
  {
    int floor = std::stoi(_msg->data());

    // Step 1: close the door.
    this->states.push_back(new CloseState(this->doorController));

    // Step 2: Move to the correct floor.
    this->states.push_back(new MoveState(floor, this->liftController));

    // Step 3: Open the door
    this->states.push_back(new OpenState(this->doorController));

    // Step 4: Wait
    this->states.push_back(new WaitState);

    // Step 5: Close the door
    this->states.push_back(new CloseState(this->doorController));
  }
  catch(...)
  {
    std::cerr << "Unable to process elevator message["
              << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void ElevatorPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->stateMutex);

  // Process the states
  if (!this->states.empty())
  {
    // Update the front state, and remove it if the state is done
    if (this->states.front()->Update())
    {
      this->states.pop_front();
    }
  }

  // Update the controllers
  this->doorController->Update();
  this->liftController->Update();
}

/////////////////////////////////////////////////
ElevatorPlugin::CloseState::CloseState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPlugin::CloseState::Start()
{
  this->ctrl->SetTarget(ElevatorPlugin::DoorController::CLOSE);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPlugin::CloseState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() == ElevatorPlugin::DoorController::CLOSE &&
      this->ctrl->GetState() == ElevatorPlugin::DoorController::STATIONARY;
  }
}

/////////////////////////////////////////////////
ElevatorPlugin::OpenState::OpenState(DoorController *_ctrl)
  : State(), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPlugin::OpenState::Start()
{
  this->ctrl->SetTarget(ElevatorPlugin::DoorController::OPEN);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPlugin::OpenState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetTarget() == ElevatorPlugin::DoorController::OPEN &&
      this->ctrl->GetState() == ElevatorPlugin::DoorController::STATIONARY;
  }
}

/////////////////////////////////////////////////
ElevatorPlugin::MoveState::MoveState(int _floor, LiftController *_ctrl)
  : State(), floor(_floor), ctrl(_ctrl)
{
}

/////////////////////////////////////////////////
void ElevatorPlugin::MoveState::Start()
{
  this->ctrl->SetFloor(this->floor);
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPlugin::MoveState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    return this->ctrl->GetState() == ElevatorPlugin::LiftController::STATIONARY;
  }
}

/////////////////////////////////////////////////
ElevatorPlugin::WaitState::WaitState()
  : State()
{
}

/////////////////////////////////////////////////
void ElevatorPlugin::WaitState::Start()
{
  this->start = common::Time::GetWallTime();
  this->started = true;
}

/////////////////////////////////////////////////
bool ElevatorPlugin::WaitState::Update()
{
  if (!this->started)
  {
    this->Start();
    return false;
  }
  else
  {
    if (common::Time::GetWallTime() - this->start > common::Time(5, 0))
      return true;
    else
      return false;
  }
}

/////////////////////////////////////////////////
ElevatorPlugin::DoorController::DoorController(physics::JointPtr _doorJoint)
  : doorJoint(_doorJoint), state(STATIONARY), target(CLOSE)
{
  this->doorPID.Init(2, 0, 1.0);
}

/////////////////////////////////////////////////
void ElevatorPlugin::DoorController::SetTarget(
    ElevatorPlugin::DoorController::Target _target)
{
  this->target = _target;
}

/////////////////////////////////////////////////
ElevatorPlugin::DoorController::Target
ElevatorPlugin::DoorController::GetTarget() const
{
  return this->target;
}

/////////////////////////////////////////////////
ElevatorPlugin::DoorController::State
ElevatorPlugin::DoorController::GetState() const
{
  return this->state;
}

/////////////////////////////////////////////////
bool ElevatorPlugin::DoorController::Update()
{
  double errorTarget = this->target == OPEN ? 1.0 : 0.0;

  double doorError = this->doorJoint->GetAngle(0).Radian() - errorTarget;
  double doorForce = this->doorPID.Update(doorError,
      common::Time(0, 1000000));
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

/////////////////////////////////////////////////
ElevatorPlugin::LiftController::LiftController(physics::JointPtr _liftJoint)
  : state(STATIONARY), floor(0), liftJoint(_liftJoint)
{
  this->liftPID.Init(100000, 0, 100000.0);
}

/////////////////////////////////////////////////
bool ElevatorPlugin::LiftController::Update()
{
  double error = this->liftJoint->GetAngle(0).Radian() -
    (this->floor * this->floorHeight) + 0.1;
  double force = this->liftPID.Update(error, common::Time(0, 1000000));

  this->liftJoint->SetForce(0, force);

  if (std::abs(error) < 0.15)
  {
    this->state = ElevatorPlugin::LiftController::STATIONARY;
    return true;
  }
  else
  {
    this->state = ElevatorPlugin::LiftController::MOVING;
    return false;
  }
}

/////////////////////////////////////////////////
void ElevatorPlugin::LiftController::SetFloor(int _floor)
{
  this->floor = _floor;
}

/////////////////////////////////////////////////
int ElevatorPlugin::LiftController::GetFloor() const
{
  return this->floor;
}

/////////////////////////////////////////////////
ElevatorPlugin::LiftController::State
ElevatorPlugin::LiftController::GetState() const
{
  return this->state;
}
