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
State::State(const std::string &_name, ValidationPlugin &_plugin)
  : name(_name),
    plugin(_plugin),
    publicationInterval(1.0)
{
  // This is the topic where we receive feedback from the controllers.
  this->node.Subscribe("/gazebo/validation/feedback", &State::OnFeedback, this);

  // Advertise the current state.
  this->node.Advertise<msgs::GzString>("/gazebo/validation/state");
}

/////////////////////////////////////////////////
void State::Update()
{
  //std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->initialized)
    this->Initialize();

  std::cout << this->name << std::endl;
  std::cout << "State::Update()" << std::endl;
  std::cout << timer.GetElapsed() << std::endl;

  this->DoUpdate();

  // Time to publish the current state?
  auto now = gazebo::common::Time::GetWallTime();
  if (now - this->lastPublication >= this->publicationInterval)
    this->PublishState();
}

/////////////////////////////////////////////////
void State::Teardown()
{
  //std::lock_guard<std::mutex> lock(this->mutex);
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
  //std::lock_guard<std::mutex> lock(this->mutex);
  return this->feedback;
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
void State::Initialize()
{
  std::cout << "State::Initialize()" << std::endl;
  this->initialized = true;
  this->timer.Reset();
  this->timer.Start();
  this->DoInitialize();
}

/////////////////////////////////////////////////
void State::OnFeedback(const msgs::GzString &_msg)
{
  std::cout << "State::OnFeedback()" << std::endl;
  //std::lock_guard<std::mutex> lock(this->mutex);
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
  this->lastPublication = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
void ReadyState::DoInitialize()
{
  std::cout << "InitState::Initialize()" << std::endl;

  // Reset the world.
}

/////////////////////////////////////////////////
void ReadyState::DoFeedback()
{
  // Is the controller ready?
  if (this->Feedback() == "controller_ready")
    this->plugin.ChangeState(*this->plugin.setState);

  std::cout << "InitiState::DoFeedback()" << std::endl;
}

/////////////////////////////////////////////////
void SetState::DoInitialize()
{
  // Load parameters.
  this->plugin.ChangeState(*this->plugin.goState);
}

/////////////////////////////////////////////////
void GoState::DoFeedback()
{
  if (this->Feedback() == "end_run")
    this->plugin.ChangeState(*this->plugin.endState);

  std::cout << "RunState::DoFeedback()" << std::endl;
}

//////////////////////////////////////////////////
ValidationPlugin::ValidationPlugin()
  : readyState(new ReadyState(kReadyState, *this)),
    setState(new SetState(kSetState, *this)),
    goState(new GoState(kGoState, *this)),
    endState(new EndState(kEndState, *this)),
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
  return false;
}
