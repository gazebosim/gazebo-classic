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
#include <gazebo/common/Time.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/transport/Node.hh>
#include "validationController.hh"

/////////////////////////////////////////////////
void ReadyState::DoInitialize()
{
  std::cout << "ReadyState::Initialize()" << std::endl;
}

/////////////////////////////////////////////////
void ReadyState::DoOnState()
{
  if (this->GazeboState() == "gazebo_go")
    this->plugin.ChangeState(*this->plugin.initCondsState);

  //std::cout << "ReadyState::DoOnState()" << std::endl;
};

/////////////////////////////////////////////////
void InitCondsState::DoInitialize()
{
  // Send initial conditions.
  std::cout << "InitCondsState::DoInitialize()" << std::endl;
  //std::cout << "Initial conditions" << std::endl;
};

/////////////////////////////////////////////////
void InitCondsState::DoUpdate()
{
  // Check if the initial conditions are satisfied.

  //std::cout << "InitCondsState::DoUpdate()" << std::endl;

  if (this->timer.GetElapsed() >= gazebo::common::Time(2.0))
    this->plugin.ChangeState(*this->plugin.runningState);
};

/////////////////////////////////////////////////
void RunningState::DoInitialize()
{
  std::cout << "RunningState::Initialize()" << std::endl;
}

/////////////////////////////////////////////////
void RunningState::DoUpdate()
{
  // Send the next command.

  //std::cout << "RunningState::DoUpdate()" << std::endl;

  // Check if we are done with the run
  if (this->timer.GetElapsed() >= gazebo::common::Time(5.0))
    this->plugin.ChangeState(*this->plugin.endState);
};

/////////////////////////////////////////////////
void EndState::DoInitialize()
{
  std::cout << "EndState::Initialize()" << std::endl;
}

/////////////////////////////////////////////////
void EndState::DoOnState()
{
  // Check if Gazebo is ready for another run.
  if (this->GazeboState() == "gazebo_ready")
    this->plugin.ChangeState(*this->plugin.readyState);

  //std::cout << "EndState::DoOnState()" << std::endl;
}

/////////////////////////////////////////////////
ValidationController::ValidationController()
  : readyState(new ReadyState(kReadyState, *this)),
    initCondsState(new InitCondsState(kInitCondsState, *this)),
    runningState(new RunningState(kRunningState, *this)),
    endState(new EndState(kEndState, *this)),
    currentState(readyState.get())
 {
 }

/////////////////////////////////////////////////
ValidationController::~ValidationController()
{
};

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
