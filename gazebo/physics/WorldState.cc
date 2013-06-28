/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
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
  : State(_world->GetName(), _world->GetRealTime(), _world->GetSimTime())
{
  this->world = _world;

  // Add a state for all the models
  Model_V models = _world->GetModels();
  for (Model_V::const_iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    if (!(*iter)->IsStatic())
    {
      this->modelStates.insert(std::make_pair((*iter)->GetName(),
            ModelState(*iter, this->realTime, this->simTime)));
    }
  }
}

/////////////////////////////////////////////////
WorldState::WorldState(const rml::ElementPtr _rml)
  : State()
{
  this->Load(_rml);
}

/////////////////////////////////////////////////
WorldState::~WorldState()
{
  this->world.reset();
  this->modelStates.clear();
}

/////////////////////////////////////////////////
void WorldState::Load(const WorldPtr _world)
{
  this->world = _world;
  this->name = _world->GetName();
  this->wallTime = common::Time::GetWallTime();
  this->simTime = _world->GetSimTime();
  this->realTime = _world->GetRealTime();

  // Add a state for all the models
  Model_V models = _world->GetModels();
  for (Model_V::const_iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    if (!(*iter)->IsStatic())
    {
      this->modelStates[(*iter)->GetName()].Load(*iter, this->realTime,
          this->simTime);
    }
  }

  // Remove models that no longer exist. We determine this by check the time
  // stamp on each model.
  for (ModelState_M::iterator iter = this->modelStates.begin();
       iter != this->modelStates.end();)
  {
    if (iter->second.GetRealTime() != this->realTime)
      this->modelStates.erase(iter++);
    else
      ++iter;
  }
}

/////////////////////////////////////////////////
void WorldState::Load(const rml::ElementPtr _elem)
{
  // Copy the name
  this->name = _elem->Get<std::string>("world_name");

  // Add the model states
  this->modelStates.clear();
  if (_elem->HasElement("model"))
  {
    rml::ElementPtr childElem = _elem->GetElement("model");

    while (childElem)
    {
      this->modelStates.insert(std::make_pair(
            childElem->Get<std::string>("name"), ModelState(childElem)));
      childElem = childElem->GetNextElement("model");
    }
  }

  // Copy the name and time information
  this->simTime = _elem->Get<common::Time>("sim_time");
  this->wallTime = _elem->Get<common::Time>("wall_time");
  this->realTime = _elem->Get<common::Time>("real_time");
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
ModelState WorldState::GetModelState(unsigned int _index) const
{
  if (_index < this->modelStates.size())
  {
    ModelState_M::const_iterator iter = this->modelStates.begin();
    std::advance(iter, _index);
    return iter->second;
  }

  gzthrow("Invalid model state index\n");
  return ModelState();
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(const std::string &_modelName) const
{
  // Search for the model name
  ModelState_M::const_iterator iter = this->modelStates.find(_modelName);
  if (iter != this->modelStates.end())
    return iter->second;

  // Throw exception if the model name doesn't exist.
  gzthrow("Invalid model name[" + _modelName + "].");
  return ModelState();
}

/////////////////////////////////////////////////
bool WorldState::HasModelState(const std::string &_modelName) const
{
  // Search for the model name
  ModelState_M::const_iterator iter = this->modelStates.find(_modelName);
  if (iter != this->modelStates.end())
    return true;

  return false;
}

/////////////////////////////////////////////////
bool WorldState::IsZero() const
{
  bool result = this->insertions.size() == 0 && this->deletions.size() == 0;

  for (ModelState_M::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end() && result; ++iter)
  {
    result = result && iter->second.IsZero();
  }

  return result;
}

/////////////////////////////////////////////////
WorldState &WorldState::operator=(const WorldState &_state)
{
  State::operator=(_state);

  // Clear the model states
  this->modelStates.clear();

  this->insertions.clear();
  this->deletions.clear();

  // Copy the model states.
  for (ModelState_M::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    this->modelStates.insert(std::make_pair(iter->first,
          ModelState(iter->second)));
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

  // Add in the new model states
  for (ModelState_M::const_iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    if (!_state.HasModelState(iter->second.GetName()) && this->world)
    {
      ModelPtr model = this->world->GetModel(iter->second.GetName());
      result.insertions.push_back(model->GetRML()->ToString(""));
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

  // Add the states.
  for (ModelState_M::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    ModelState state = this->GetModelState(iter->second.GetName()) +
      iter->second;
    result.modelStates.insert(std::make_pair(state.GetName(), state));
  }

  return result;
}

/////////////////////////////////////////////////
void WorldState::FillRML(rml::ElementPtr _rml)
{
  _rml->ClearElements();

  _rml->GetAttribute("world_name")->Set(this->name);
  _rml->GetElement("sim_time")->Set(this->simTime);
  _rml->GetElement("real_time")->Set(this->realTime);
  _rml->GetElement("wall_time")->Set(this->wallTime);

  for (ModelState_M::iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    rml::ElementPtr elem = _rml->AddElement("model");
    iter->second.FillRML(elem);
  }
}

/////////////////////////////////////////////////
void WorldState::SetWallTime(const common::Time &_time)
{
  State::SetWallTime(_time);

  for (ModelState_M::iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetWallTime(_time);
  }
}

/////////////////////////////////////////////////
void WorldState::SetRealTime(const common::Time &_time)
{
  State::SetRealTime(_time);

  for (ModelState_M::iterator iter = this->modelStates.begin();
           iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetRealTime(_time);
  }
}

/////////////////////////////////////////////////
void WorldState::SetSimTime(const common::Time &_time)
{
  State::SetSimTime(_time);

  for (ModelState_M::iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    iter->second.SetSimTime(_time);
  }
}
