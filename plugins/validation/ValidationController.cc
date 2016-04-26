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
#include <ignition/transport/Node.hh>
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/msgs/msgs.hh"
#include "ValidationController.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
ValidationController::ValidationController()
  : readyState(new ControllerReadyState<ValidationController>(kControllerReadyState, *this)),
    initCondsState(new ControllerInitCondsState<ValidationController>(kControllerInitCondsState, *this)),
    runningState(new ControllerRunningState<ValidationController>(kControllerRunningState, *this)),
    endState(new ControllerEndState<ValidationController>(kControllerEndState, *this)),
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
void ValidationController::ChangeState(State<ValidationController> &_newState)
{
  // Only update the state if _newState is different than the current state.
  if (!this->currentState || *this->currentState != _newState)
  {
    this->currentState->Teardown();
    this->currentState = &_newState;
  }
}
