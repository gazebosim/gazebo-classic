/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "SimStateEventSource.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
SimStateEventSource::SimStateEventSource(transport::PublisherPtr _pub,
                                         physics::WorldPtr _world)
  :EventSource(_pub, "sim_state", _world), hasPaused(false)
{
}

////////////////////////////////////////////////////////////////////////////////
SimStateEventSource::~SimStateEventSource()
{
  this->pauseConnection.reset();
}

////////////////////////////////////////////////////////////////////////////////
void SimStateEventSource::Load(const sdf::ElementPtr _sdf)
{
  EventSource::Load(_sdf);

  // Listen to the pause event. This event is broadcast every
  // simulation iteration.
  this->pauseConnection = event::Events::ConnectPause(
      boost::bind(&SimStateEventSource::OnPause, this, _1));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&SimStateEventSource::OnUpdate, this, _1));
}

////////////////////////////////////////////////////////////////////////////////
void SimStateEventSource::OnUpdate(const common::UpdateInfo &_info)
{
  if (_info.simTime < this->simTime)
  {
    std::string json;
    json = "{\"state\": \"reset\" }";
    this->Emit(json);
  }
  this->simTime = _info.simTime;
}

////////////////////////////////////////////////////////////////////////////////
void SimStateEventSource::OnPause(const bool _pause)
{
  std::string json;
  if (_pause)
  {
    json = "{\"state\": \"paused\" }";
  }
  else
  {
    json = "{\"state\": \"running\" }";
  }
  this->Emit(json);
  this->hasPaused = _pause;
}
