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
  : State(_world->GetName(), _world->GetSimTime(), _world->GetRealTime())
{
  this->world = _world;

  // Add a state for all the models
  Model_V models = _world->GetModels();
  for (Model_V::const_iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    this->modelStates.push_back(ModelState(*iter));
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
}

/////////////////////////////////////////////////
void WorldState::Load(const sdf::ElementPtr _elem)
{
  // Copy the name and time information
  this->name = _elem->GetValueString("world_name");
  this->simTime = _elem->GetValueTime("sim_time");
  this->wallTime = _elem->GetValueTime("wall_time");
  this->realTime = _elem->GetValueTime("real_time");

  // Add the model states
  this->modelStates.clear();
  if (_elem->HasElement("model"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("model");

    while (childElem)
    {
      this->modelStates.push_back(ModelState(childElem));
      childElem = childElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void WorldState::SetWorld(const WorldPtr _world)
{
  this->world = _world;
}

/////////////////////////////////////////////////
const std::vector<ModelState> &WorldState::GetModelStates() const
{
  return this->modelStates;
}

/////////////////////////////////////////////////
unsigned int WorldState::GetModelStateCount() const
{
  return this->modelStates.size();
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(unsigned int _index) const
{
  // Check to see if the _index is valid.
  if (_index < this->modelStates.size())
    return this->modelStates[_index];
  else
    gzerr << "Index is out of range\n";

  return ModelState();
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(const std::string &_modelName) const
{
  // Search for the model name
  for (std::vector<ModelState>::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if ((*iter).GetName() == _modelName)
      return *iter;
  }

  // Throw exception if the model name doesn't exist.
  gzthrow("Invalid model name[" + _modelName + "].");
  return ModelState();
}

/////////////////////////////////////////////////
bool WorldState::HasModelState(const std::string &_modelName) const
{
  // Search for the model name
  for (std::vector<ModelState>::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if ((*iter).GetName() == _modelName)
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
bool WorldState::IsZero() const
{
  bool result = this->newModels.size() == 0;

  for (std::vector<ModelState>::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end() && result; ++iter)
  {
    result = result && (*iter).IsZero();
  }

  return result;
}

/////////////////////////////////////////////////
WorldState &WorldState::operator=(const WorldState &_state)
{
  State::operator=(_state);

  // Clear the model states
  this->modelStates.clear();

  this->newModels.clear();

  // Copy the model states.
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    this->modelStates.push_back(ModelState(*iter));
  }

  // Copy the newModel states.
  for (std::vector<std::string>::const_iterator iter =
       _state.newModels.begin(); iter != _state.newModels.end(); ++iter)
  {
    this->newModels.push_back(*iter);
  }

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
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    if (this->HasModelState((*iter).GetName()))
    {
      ModelState state = this->GetModelState((*iter).GetName()) - *iter;

      if (!state.IsZero())
      {
        result.modelStates.push_back(state);
      }
    }
    else
    {
      std::cout << "[" << (*iter).GetName() << "]\n";
      if ((*iter).GetName().empty())
        printf("BAD\n");
      printf("HERE\n");
      // result.modelStates.push_back(*iter);
    }
  }

  // Add in the new model states
  for (std::vector<ModelState>::const_iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    if (!_state.HasModelState((*iter).GetName()))
    {
      result.modelStates.push_back(*iter);

      if (this->world)
      {
        printf("First\n");
        ModelPtr model = this->world->GetModel((*iter).GetName());
        result.newModels.push_back(model->GetSDF()->ToString(""));
      }
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
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    ModelState state = this->GetModelState((*iter).GetName()) + *iter;
    result.modelStates.push_back(state);
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

  for (std::vector<ModelState>::iterator iter =
       this->modelStates.begin(); iter != this->modelStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("model");
    (*iter).FillSDF(elem);
  }
}
