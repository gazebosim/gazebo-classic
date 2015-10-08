/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: A world state
 * Author: Nate Koenig
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/WorldState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
WorldState::WorldState()
  : State()
{
}

/////////////////////////////////////////////////
WorldState::WorldState(const WorldPtr _world)
  : State(_world->GetName(), _world->GetRealTime(), _world->GetSimTime(),
      _world->GetIterations())
{
  this->world = _world;

  // Add a state for all the models
  Model_V models = _world->GetModels();
  for (Model_V::const_iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    this->modelStates.insert(std::make_pair((*iter)->GetName(),
          ModelState(*iter, this->realTime, this->simTime, this->iterations)));
  }

  // Add states for all the lights
  for (const auto &light : _world->Lights())
  {
    this->lightStates.insert(std::make_pair(light->GetName(),
          LightState(light, this->realTime, this->simTime, this->iterations)));
  }
}

/////////////////////////////////////////////////
WorldState::WorldState(const sdf::ElementPtr _sdf)
  : State()
{
  this->Load(_sdf);
}

/////////////////////////////////////////////////
WorldState::~WorldState()
{
  this->world.reset();
  this->modelStates.clear();
  this->lightStates.clear();
}

/////////////////////////////////////////////////
void WorldState::Load(const WorldPtr _world)
{
  this->world = _world;
  this->name = _world->GetName();
  this->wallTime = common::Time::GetWallTime();
  this->simTime = _world->GetSimTime();
  this->realTime = _world->GetRealTime();
  this->iterations = _world->GetIterations();

  // Add a state for all the models
  this->modelStates.clear();
  Model_V models = _world->GetModels();
  for (Model_V::const_iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    this->modelStates[(*iter)->GetName()].Load(*iter, this->realTime,
        this->simTime, this->iterations);
  }

  // Add states for all the lights
  this->lightStates.clear();
  Light_V lights = _world->Lights();
  for (const auto &light : lights)
  {
    this->lightStates[light->GetName()].Load(light, this->realTime,
        this->simTime, this->iterations);
  }
}

/////////////////////////////////////////////////
void WorldState::Load(const sdf::ElementPtr _elem)
{
  // Copy the name and time information
  this->name = _elem->Get<std::string>("world_name");
  this->simTime = _elem->Get<common::Time>("sim_time");
  this->wallTime = _elem->Get<common::Time>("wall_time");
  this->realTime = _elem->Get<common::Time>("real_time");
  this->iterations = _elem->Get<uint64_t>("iterations");

  // Add the model states
  this->modelStates.clear();
  if (_elem->HasElement("model"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("model");

    while (childElem)
    {
      std::string modelName = childElem->Get<std::string>("name");
      this->modelStates.insert(std::make_pair(
            modelName, ModelState(childElem)));
      this->modelStates[modelName].SetSimTime(this->simTime);
      this->modelStates[modelName].SetWallTime(this->wallTime);
      this->modelStates[modelName].SetRealTime(this->realTime);
      this->modelStates[modelName].SetIterations(this->iterations);
      childElem = childElem->GetNextElement("model");
    }
  }

  // Add the light states
  this->lightStates.clear();
  if (_elem->HasElement("light"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("light");

    while (childElem)
    {
      LightState lightState(childElem);
      lightState.SetSimTime(this->simTime);
      lightState.SetWallTime(this->wallTime);
      lightState.SetRealTime(this->realTime);
      lightState.SetIterations(this->iterations);

      std::string lightName = childElem->Get<std::string>("name");
      this->lightStates.insert(std::make_pair(
            lightName, lightState));

      childElem = childElem->GetNextElement("light");
    }
  }
}

/////////////////////////////////////////////////
void WorldState::SetWorld(const WorldPtr _world)
{
  this->world = _world;
}

/////////////////////////////////////////////////
const ModelState_M &WorldState::GetModelStates() const
{
  return this->modelStates;
}

/////////////////////////////////////////////////
const LightState_M &WorldState::LightStates() const
{
  return this->lightStates;
}

/////////////////////////////////////////////////
ModelState_M WorldState::GetModelStates(const boost::regex &_regex) const
{
  ModelState_M result;

  // Search for matching link names
  for (ModelState_M::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if (boost::regex_match(iter->first, _regex))
      result.insert(std::make_pair(iter->first, iter->second));
  }

  return result;
}

/////////////////////////////////////////////////
unsigned int WorldState::GetModelStateCount() const
{
  return this->modelStates.size();
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(const std::string &_modelName) const
{
  // Search for the model name
  ModelState_M::const_iterator iter = this->modelStates.find(_modelName);
  if (iter != this->modelStates.end())
    return iter->second;

  // Error if the model name doesn't exist.
  gzerr << "Invalid model name[" + _modelName + "]." << std::endl;
  return ModelState();
}

/////////////////////////////////////////////////
LightState WorldState::GetLightState(const std::string &_lightName) const
{
  // Search for the light name
  auto iter = this->lightStates.find(_lightName);
  if (iter != this->lightStates.end())
    return iter->second;

  // Error if the light name doesn't exist.
  gzerr << "Invalid light name[" + _lightName + "]." << std::endl;
  return LightState();
}

/////////////////////////////////////////////////
bool WorldState::HasModelState(const std::string &_modelName) const
{
  return this->modelStates.find(_modelName) != this->modelStates.end();
}

/////////////////////////////////////////////////
bool WorldState::HasLightState(const std::string &_lightName) const
{
  return this->lightStates.find(_lightName) != this->lightStates.end();
}

/////////////////////////////////////////////////
bool WorldState::IsZero() const
{
  bool result = this->insertions.size() == 0 && this->deletions.size() == 0;

  // Model
  for (ModelState_M::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end() && result; ++iter)
  {
    result = result && iter->second.IsZero();
  }

  // Light
  for (const auto &light : this->lightStates)
  {
    result = result && light.second.IsZero();
  }

  return result;
}

/////////////////////////////////////////////////
WorldState &WorldState::operator=(const WorldState &_state)
{
  State::operator=(_state);

  // Clear the states
  this->modelStates.clear();
  this->lightStates.clear();

  this->insertions.clear();
  this->deletions.clear();

  // Copy the model states.
  for (ModelState_M::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    this->modelStates.insert(std::make_pair(iter->first,
          ModelState(iter->second)));
  }

  // Copy the light states.
  for (const auto &light : this->lightStates)
  {
    this->lightStates.insert(std::make_pair(light.first,
          LightState(light.second)));
  }

  // Copy the insertions
  std::copy(_state.insertions.begin(),
            _state.insertions.end(), this->insertions.begin());

  // Copy the deletions
  std::copy(_state.deletions.begin(),
            _state.deletions.end(), this->deletions.begin());

  return *this;
}

/////////////////////////////////////////////////
WorldState WorldState::operator-(const WorldState &_state) const
{
  WorldState result;

  result.name = this->name;
  result.simTime = this->simTime;
  result.realTime = this->realTime;
  result.wallTime = this->wallTime;
  result.iterations = this->iterations;

  // Subtract the model states.
  for (ModelState_M::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    if (this->HasModelState(iter->second.GetName()))
    {
      ModelState state = this->GetModelState(iter->second.GetName()) -
        iter->second;

      if (!state.IsZero())
      {
        result.modelStates.insert(std::make_pair(state.GetName(), state));
      }
    }
    else
    {
      result.deletions.push_back(iter->second.GetName());
    }
  }

  // Subtract the light states.
  for (const auto &light : this->lightStates)
  {
    if (this->HasLightState(light.second.GetName()))
    {
      LightState state = this->GetLightState(light.second.GetName()) -
          light.second;

      if (!state.IsZero())
      {
        result.lightStates.insert(std::make_pair(state.GetName(), state));
      }
    }
    else
    {
      result.deletions.push_back(light.second.GetName());
    }
  }

  // Add in the new model states
  for (ModelState_M::const_iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    if (!_state.HasModelState(iter->second.GetName()) && this->world)
    {
      ModelPtr model = this->world->GetModel(iter->second.GetName());
      result.insertions.push_back(model->GetSDF()->ToString(""));
    }
  }

  // Add in the new light states
  for (const auto &light : this->lightStates)
  {
    if (!_state.HasLightState(light.second.GetName()) && this->world)
    {
      LightPtr lightPtr = this->world->Light(light.second.GetName());
      result.insertions.push_back(lightPtr->GetSDF()->ToString(""));
    }
  }

  return result;
}

/////////////////////////////////////////////////
WorldState WorldState::operator+(const WorldState &_state) const
{
  WorldState result;

  result.name = this->name;
  result.simTime = this->simTime;
  result.realTime = this->realTime;
  result.wallTime = this->wallTime;
  result.iterations = this->iterations;

  // Add the model states.
  for (ModelState_M::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    ModelState state = this->GetModelState(iter->second.GetName()) +
      iter->second;
    result.modelStates.insert(std::make_pair(state.GetName(), state));
  }

  // Add the light states.
  for (const auto &light : this->lightStates)
  {
    LightState state = this->GetLightState(light.second.GetName()) +
        light.second;
    result.lightStates.insert(std::make_pair(state.GetName(), state));
  }

  return result;
}

/////////////////////////////////////////////////
void WorldState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("world_name")->Set(this->name);
  _sdf->GetElement("sim_time")->Set(this->simTime);
  _sdf->GetElement("real_time")->Set(this->realTime);
  _sdf->GetElement("wall_time")->Set(this->wallTime);
  _sdf->GetElement("iterations")->Set(this->iterations);

  // Models
  for (ModelState_M::iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("model");
    iter->second.FillSDF(elem);
  }

  // Lights
  for (auto &light : this->lightStates)
  {
    sdf::ElementPtr elem = _sdf->AddElement("light");
    light.second.FillSDF(elem);
  }
}

/////////////////////////////////////////////////
void WorldState::SetWallTime(const common::Time &_time)
{
  State::SetWallTime(_time);

  // Models
  for (ModelState_M::iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetWallTime(_time);
  }

  // Lights
  for (auto &light : this->lightStates)
  {
    light.second.SetWallTime(_time);
  }
}

/////////////////////////////////////////////////
void WorldState::SetRealTime(const common::Time &_time)
{
  State::SetRealTime(_time);

  // Models
  for (ModelState_M::iterator iter = this->modelStates.begin();
           iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetRealTime(_time);
  }

  // Lights
  for (auto &light : this->lightStates)
  {
    light.second.SetRealTime(_time);
  }
}

/////////////////////////////////////////////////
void WorldState::SetSimTime(const common::Time &_time)
{
  State::SetSimTime(_time);

  // Models
  for (ModelState_M::iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetSimTime(_time);
  }

  // Lights
  for (auto &light : this->lightStates)
  {
    light.second.SetSimTime(_time);
  }
}

/////////////////////////////////////////////////
void WorldState::SetIterations(const uint64_t _iterations)
{
  State::SetIterations(_iterations);

  // Models
  for (auto &modelState : this->modelStates)
    modelState.second.SetIterations(_iterations);

  // Lights
  for (auto &lightState : this->lightStates)
    lightState.second.SetIterations(_iterations);
}
