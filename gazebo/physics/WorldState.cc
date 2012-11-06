/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/WorldState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
WorldState::WorldState()
  : State()
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->sdf);
}

/////////////////////////////////////////////////
WorldState::WorldState(WorldPtr _world)
  : State(_world->GetName(), _world->GetSimTime(), _world->GetRealTime())
{
  for (unsigned int i = 0; i < _world->GetModelCount(); ++i)
  {
    ModelPtr model = _world->GetModel(i);
    if (model)
      this->modelStates.push_back(ModelState(model));
  }
}

/////////////////////////////////////////////////
void WorldState::UpdateSDF()
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->sdf);

  this->sdf->GetAttribute("world_name")->Set(this->GetName());

  this->sdf->GetElement("sim_time")->Set(this->GetSimTime());
  this->sdf->GetElement("wall_time")->Set(this->GetWallTime());
  this->sdf->GetElement("real_time")->Set(this->GetRealTime());

  for (std::vector<ModelState>::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    sdf::ElementPtr modelElem = this->sdf->AddElement("model");
    (*iter).FillStateSDF(modelElem);
  }
}

/////////////////////////////////////////////////
void WorldState::UpdateSDF(WorldPtr /*_world*/)
{
  /*
  this->sdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->sdf);

  this->sdf->GetElement("sim_time")->Set(_world->GetSimTime());
  this->sdf->GetElement("wall_time")->Set(common::Time::GetWallTime());
  this->sdf->GetElement("real_time")->Set(_world->GetRealTime());

  for (unsigned int i = 0; i < _world->GetModelCount(); ++i)
  {
    sdf::ElementPtr modelElem = this->sdf->AddElement("model");
    this->modelStates.push_back(ModelState(_world->GetModel(i)));
    this->modelStates.back().FillStateSDF(modelElem);
  }
  */
}

/////////////////////////////////////////////////
WorldState::~WorldState()
{
  this->sdf.reset();
  this->modelStates.clear();
}

/////////////////////////////////////////////////
void WorldState::Load(sdf::ElementPtr _elem)
{
  this->name = _elem->GetValueString("world_name");
  this->simTime = _elem->GetValueTime("sim_time");
  this->wallTime = _elem->GetValueTime("wall_time");
  this->realTime = _elem->GetValueTime("real_time");

  if (_elem->HasElement("model"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("model");

    while (childElem)
    {
      ModelState state;
      state.Load(childElem);
      this->modelStates.push_back(state);
      childElem = childElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
unsigned int WorldState::GetModelStateCount() const
{
  return this->modelStates.size();
}

/////////////////////////////////////////////////
const sdf::ElementPtr &WorldState::GetSDF() const
{
  return this->sdf;
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(unsigned int _index) const
{
  if (_index < this->modelStates.size())
    return this->modelStates[_index];
  else
    gzerr << "Index is out of range\n";

  return ModelState();
}

/////////////////////////////////////////////////
ModelState WorldState::GetModelState(const std::string &_modelName) const
{
  std::vector<ModelState>::const_iterator iter;

  for (iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if ((*iter).GetName() == _modelName)
      return *iter;
  }

  return ModelState();
}

/////////////////////////////////////////////////
bool WorldState::HasModelState(const std::string &_modelName) const
{
  std::vector<ModelState>::const_iterator iter;

  for (iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if ((*iter).GetName() == _modelName)
      return true;
  }

  return false;
}


/////////////////////////////////////////////////
WorldState &WorldState::operator=(const WorldState &_state)
{
  State::operator=(_state);

  // Clear the model states
  this->modelStates.clear();

  // Copy the model states.
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    this->modelStates.push_back(ModelState(*iter));
  }

  // Copy the SDF values
  this->sdf = _state.sdf;

  return *this;
}

/////////////////////////////////////////////////
WorldState WorldState::operator-(const WorldState &_state) const
{
  WorldState result = *this;

  result.simTime -= _state.simTime;
  result.realTime -= _state.realTime;
  result.wallTime -= _state.wallTime;

  result.modelStates.clear();
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    if (this->HasModelState((*iter).GetName()))
    {
      ModelState state = this->GetModelState((*iter).GetName()) - *iter;
      if (!state.IsZero())
        result.modelStates.push_back(state);
    }
    else
      result.modelStates.push_back(*iter);
  }

  return result;
}

/////////////////////////////////////////////////
WorldState WorldState::operator+(const WorldState &_state) const
{
  WorldState result = *this;

  result.simTime += _state.simTime;
  result.realTime += _state.realTime;
  result.wallTime += _state.wallTime;

  result.modelStates.clear();
  for (std::vector<ModelState>::const_iterator iter =
       _state.modelStates.begin(); iter != _state.modelStates.end(); ++iter)
  {
    ModelState state = this->GetModelState((*iter).GetName()) + *iter;
    result.modelStates.push_back(state);
  }

  return result;
}

/////////////////////////////////////////////////
bool WorldState::IsZero() const
{
  bool result = true;

  for (std::vector<ModelState>::const_iterator iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    result = result && (*iter).IsZero();
  }

  return result;
}
