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

#include <mutex>
#include <string>
#include "State.hh"
#include "ValidationController.hh"

using namespace gazebo;

//////////////////////////////////////////////////
ControllerState::ControllerState(const std::string &_name,
    ValidationController &_controller)
  : State(_name),
    controller(_controller)
{
}

/////////////////////////////////////////////////
void ControllerReadyState::DoFeedback()
{
  if (this->Feedback() == "gazebo_go")
    this->controller.ChangeState(*this->controller.initCondsState);

  //std::cout << "ReadyState::DoOnState()" << std::endl;
}

/////////////////////////////////////////////////
void ControllerInitCondsState::DoInitialize()
{
  // Send initial conditions.
  std::cout << "InitCondsState::DoInitialize()" << std::endl;
  //std::cout << "Initial conditions" << std::endl;
}

/////////////////////////////////////////////////
void ControllerInitCondsState::DoUpdate()
{
  // Check if the initial conditions are satisfied.

  //std::cout << "InitCondsState::DoUpdate()" << std::endl;

  if (this->timer.GetElapsed() >= gazebo::common::Time(2.0))
    this->controller.ChangeState(*this->controller.runningState);
}

/////////////////////////////////////////////////
void ControllerRunningState::DoInitialize()
{
  std::cout << "RunningState::Initialize()" << std::endl;
}

/////////////////////////////////////////////////
void ControllerRunningState::DoUpdate()
{
  // Send the next command.

  //std::cout << "RunningState::DoUpdate()" << std::endl;

  // Check if we are done with the run
  if (this->timer.GetElapsed() >= gazebo::common::Time(5.0))
    this->controller.ChangeState(*this->controller.endState);
}

/////////////////////////////////////////////////
void ControllerEndState::DoInitialize()
{
  std::cout << "EndState::Initialize()" << std::endl;
}

/////////////////////////////////////////////////
void ControllerEndState::DoFeedback()
{
  // Check if Gazebo is ready for another run.
  if (this->Feedback() == "gazebo_ready")
    this->controller.ChangeState(*this->controller.readyState);

  //std::cout << "EndState::DoOnState()" << std::endl;
}

/////////////////////////////////////////////////
ValidationController::ValidationController()
  : readyState(new ControllerReadyState(kControllerReadyState, *this)),
    initCondsState(new ControllerInitCondsState(kControllerInitCondsState, *this)),
    runningState(new ControllerRunningState(kControllerRunningState, *this)),
    endState(new ControllerEndState(kControllerEndState, *this)),
    currentState(readyState.get())
 {
 }

/////////////////////////////////////////////////
ValidationController::~ValidationController()
{
}

/////////////////////////////////////////////////
void ValidationController::Start()
{
  while (true)
  {
    if (this->currentState)
      this->currentState->Update();

    // 1000 Hz.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

//////////////////////////////////////////////////
void ValidationController::ChangeState(State &_newState)
{
  // Only update the state if _newState is different than the current state.
  if (!this->currentState || *this->currentState != _newState)
  {
    this->currentState->Teardown();
    this->currentState = &_newState;
  }
}
