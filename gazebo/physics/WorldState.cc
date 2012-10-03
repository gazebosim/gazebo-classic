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

#include "World.hh"
#include "Model.hh"
#include "WorldState.hh"

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
  this->sdf.reset(new sdf::Element);
  sdf::initFile("state.sdf", this->sdf);

  this->sdf->GetElement("time")->Set(_world->GetSimTime());

  for (unsigned int i = 0; i < _world->GetModelCount(); ++i)
  {
    sdf::ElementPtr modelElem = this->sdf->AddElement("model");
    this->modelStates.push_back(_world->GetModel(i)->GetState());
    this->modelStates.back().FillStateSDF(modelElem);
  }
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
