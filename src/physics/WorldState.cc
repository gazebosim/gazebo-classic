/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 */

#include "WorldState.hh"
using namespace gazebo;
using namespace physics;

WorldState::WorldState()
{
}

WorldState::WorldState(const WorldPtr _world)
: wallTime(common::Time::GetWallTime), simTime(_world->GetSimTime()),
  realTime(_world->GetRealTime())
{
  for (unsigned int i = 0; i < _world->GetModelCount(); ++i)
  {
    ModelPtr model = _world->GetModel(i);
    this->modelStates.push_back(model->GetState());
  }
}

virtual WorldState::~WorldState()
{
  this->modelStates.clear()
}

common::Time WorldState::GetWallTime() const
{
  return this->wallTime;
}

common::Time WorldState::GetRealTime() const
{
  return this->realTime;
}

common::Time WorldState::GetSimTime() const
{
  return this->simTime;
}

unsigned int WorldState::GetModelStateCount() const
{
  return this->modelStates.size();
}

const ModelState &WorldState::GetModelStateCount(unsigned int _index) const
{
  if (_index < this->modelStates.size())
    return this->modelStates[_index];
  else
    gzerr << "Index is out of range\n";
}

ModelState WorldState::GetModelStateCount(const std::string &_modelName) const
{
  for (iter = this->modelStates.begin();
       iter != this->modelStates.end(); ++iter)
  {
    if ((*iter)->GetName() == _modelName)
      return *iter;
  }

  return ModelState();
}
