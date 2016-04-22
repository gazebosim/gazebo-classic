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
#include "gazebo/msgs/gz_string.pb.h"
#include "ValidationPlugin.hh"

using namespace gazebo;
using namespace ignition;

/////////////////////////////////////////////////
State::State(const std::string &_name, ValidationPlugin &_plugin):
  name(_name),
  plugin(_plugin),
  publicationInterval(1.0)
{
  // This is the topic where we receive feedback from the controllers.
  this->node.Subscribe("/gazebo/validation/feedback", &State::OnFeedback, this);

  // Advertise the current state.
  this->node.Advertise<msgs::GzString>("/gazebo/validation/state");
}

/////////////////////////////////////////////////
void State::Initialize()
{
  std::cout << "State::Initialize()" << std::endl;
  this->initialized = true;
  this->timer.Reset();
  this->timer.Start();
  this->DoInitialize();
}

/////////////////////////////////////////////////
void State::Update()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->initialized)
    this->Initialize();

  std::cout << this->name << std::endl;
  std::cout << "State::Update()" << std::endl;
  std::cout << timer.GetElapsed() << std::endl;

  this->DoUpdate();

  // Time to publish the current state?
  auto now = physics::get_world()->GetSimTime();
  if (now - this->lastPublication >= this->publicationInterval)
    this->PublishState();
}

/////////////////////////////////////////////////
void State::Teardown()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->initialized = false;

  std::cout << "State::Teardown()" << std::endl;
  this->DoTeardown();

  this->PublishState();
}

/////////////////////////////////////////////////
bool State::operator ==(const State &_state) const
{
  return this->name == _state.name;
}

/////////////////////////////////////////////////
bool State::operator !=(const State &_state) const
{
  return !(*this == _state);
}

/////////////////////////////////////////////////
std::string State::Feedback() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->feedback;
}

/////////////////////////////////////////////////
void State::OnFeedback(const msgs::GzString &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->feedback = _msg.data();

  // Only if we're the active state.
  if (this->initialized)
    this->DoFeedback();
}

/////////////////////////////////////////////////
void State::PublishState()
{
  msgs::GzString msg;
  msg.set_data(this->name);
  this->node.Publish("/gazebo/validation/state", msg);
  this->lastPublication = physics::get_world()->GetSimTime();
}

/////////////////////////////////////////////////
void State::DoInitialize()
{
  std::cout << "State::DoInitialize" << std::endl;
}

/////////////////////////////////////////////////
void State::DoUpdate()
{
  std::cout << "State::DoUpdate" << std::endl;
}

/////////////////////////////////////////////////
void State::DoTeardown()
{
  std::cout << "State::DoTeardown" << std::endl;
}

/////////////////////////////////////////////////
void State::DoFeedback()
{
  std::cout << "State::DoFeedback" << std::endl;
}

/////////////////////////////////////////////////
void InitState::DoInitialize()
{
  std::cout << "InitState::Initialize()" << std::endl;

  // Reset the world.

  // Load parameters.

}

/////////////////////////////////////////////////
void InitState::DoFeedback()
{
  // Is the controller ready?
  if (this->Feedback() == "ready")
    this->plugin.SetState(*this->plugin.setupState);

  std::cout << "InitiState::DoFeedback()" << std::endl;
}

/////////////////////////////////////////////////
void SetupState::DoFeedback()
{
  // Are the initial conditions ready?
  if (this->Feedback() == "initial_conditions")
    this->plugin.SetState(*this->plugin.runState);

  std::cout << "SetupState::DoFeedback()" << std::endl;
}

/////////////////////////////////////////////////
void RunState::DoFeedback()
{
  if (this->Feedback() == "end_run")
    this->plugin.SetState(*this->plugin.evalState);

  std::cout << "RunState::DoFeedback()" << std::endl;
}

/////////////////////////////////////////////////
void EvalState::DoFeedback()
{
  if (this->Feedback() == "end_eval")
    this->plugin.SetState(*this->plugin.stopState);

  std::cout << "EvalState::DoFeedback()" << std::endl;
}

/////////////////////////////////////////////////
void StopState::DoInitialize()
{
  // Check if this was the last run.
  if (this->plugin.MoreRuns())
    this->plugin.SetState(*this->plugin.initState);
}

//////////////////////////////////////////////////
ValidationPlugin::ValidationPlugin()
  : initState(new InitState(kInitState, *this)),
    setupState(new SetupState(kSetupState, *this)),
    runState(new RunState(kRunState, *this)),
    evalState(new EvalState(kEvalState, *this)),
    stopState(new StopState(kStopState, *this)),
    currentState(initState.get())
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
void ValidationPlugin::SetState(State &_newState)
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
  return false;
}
