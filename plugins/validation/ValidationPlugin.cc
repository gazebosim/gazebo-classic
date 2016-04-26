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
#include "gazebo/common/Console.hh"
#include "State.hh"
#include "ValidationPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
GazeboState::GazeboState(const std::string &_name,
    ValidationPlugin &_plugin)
  : State(_name, ValidationComponent_t::GAZEBO),
    plugin(_plugin)
{
}

//////////////////////////////////////////////////
void GazeboReadyState::DoFeedback()
{
  // Is the controller ready?
  if (this->Feedback() == "controller_ready")
    this->plugin.ChangeState(*this->plugin.setState);

  //std::cout << "ReadyState::DoFeedback()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboSetState::DoInitialize()
{
  // Load the parameters.

  // Start the run.
  this->plugin.ChangeState(*this->plugin.goState);

  std::cout << "SetState::DoInitialize()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboGoState::DoInitialize()
{
  std::cout << "GoState::DoInitialize()" << std::endl;
}


//////////////////////////////////////////////////
void GazeboGoState::DoFeedback()
{
  if (this->Feedback() == "controller_end")
  {
    if (this->plugin.MoreRuns())
    {
      // Go for the next run.
      this->plugin.ChangeState(*this->plugin.readyState);
    }
    else
    {
      this->plugin.ChangeState(*this->plugin.endState);
    }
  }

  // std::cout << "RunState::DoFeedback()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboEndState::DoInitialize()
{
  std::cout << "EndState::DoInitialize()" << std::endl;
}

//////////////////////////////////////////////////
ValidationPlugin::ValidationPlugin()
  : readyState(new GazeboReadyState(kReadyState, *this)),
    setState(new GazeboSetState(kSetState, *this)),
    goState(new GazeboGoState(kGoState, *this)),
    endState(new GazeboEndState(kEndState, *this)),
    currentState(readyState.get())
{
  std::cout << "Validation plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void ValidationPlugin::Load(physics::ModelPtr /*_parent*/,
   sdf::ElementPtr /*_sdf*/)
{
  // Set up a physics update callback
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ValidationPlugin::WorldUpdateCallback, this));
}

//////////////////////////////////////////////////
void ValidationPlugin::WorldUpdateCallback()
{
  if (this->currentState)
    this->currentState->Update();
}

//////////////////////////////////////////////////
bool ValidationPlugin::LoadModelParams()
{
  return true;
}

//////////////////////////////////////////////////
void ValidationPlugin::ChangeState(State &_newState)
{
  // Only update the state if _newState is different than the current state.
  if (!this->currentState || *this->currentState != _newState)
  {
    this->currentState->Teardown();
    this->currentState = &_newState;
  }
}

//////////////////////////////////////////////////
bool ValidationPlugin::MoreRuns() const
{
  static int counter = 0;
  ++counter;

  return counter <= 3;
}
