/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <boost/any.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

PresetManager::PresetManager(WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->dataPtr->physicsEngine = _world->GetPhysicsEngine();

  // Load SDF
  if (_sdf->HasElement("presets")
  {
  }
}

PresetManager::~PresetManager()
{
}

bool PresetManager::SetCurrentProfile(const std::string& _name)
{
  if (this->dataPtr->presetProfiles.find(_name) ==
      this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Profile " << _name << " not found." << std::endl;
    return false;
  }
  this->dataPtr->currentPreset = this->dataPtr->presetProfiles[_name];

  bool result = true;
  for (Preset::iterator it = this->dataPtr->currentPreset->begin();
     it != this->dataPtr->currentPreset->end(); ++it)
  {
    result = result &&
      this->dataPtr->physicsEngine->SetParam(it->first, it->second);
  }
  return result;
}

Preset* PresetManager::GetCurrentProfile()
{
  return this->dataPtr->currentPreset;
}

// name can be an entry in preset
std::string PresetManager::GetCurrentProfileName()
{
  if (!this->dataPtr->currentPreset)
  {
    return "";
  }
  // TODO: type checking
  return boost::any_cast<std::string>(
    this->dataPtr->currentPreset->at("name"));
}

std::vector<Preset*> PresetManager::GetAllProfiles()
{
  std::vector<Preset*> ret;
  for (std::map<std::string, Preset*>::iterator it = this->dataPtr->presetProfiles.begin();
     it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(it->second);
  }
  return ret;
}

std::vector<std::string> PresetManager::GetAllProfileNames()
{
  std::vector<std::string> ret;
  for (std::map<std::string, Preset*>::iterator it =
        this->dataPtr->presetProfiles.begin();
         it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(it->first);
  }
  return ret;
}

bool PresetManager::SetProfileParam(const std::string& _profileName,
  const std::string& _key, const boost::any &_value)
{
  if (_profileName == this->GetCurrentProfileName())
  {
    return this->SetCurrentProfileParam(_key, _value);
  }

  // TODO: error cond?
  this->dataPtr->presetProfiles[_profileName]->at(_key) = _value;
  return true;
}

bool PresetManager::SetCurrentProfileParam(const std::string& _key,
  const boost::any &_value)
{
  if (!this->dataPtr->currentPreset)
  {
    return false;
  }
  this->dataPtr->currentPreset->at(_key) = _value;
  return this->dataPtr->physicsEngine->SetParam(_key, _value);
}

void PresetManager::CreateProfile(const std::string& _name)
{
  this->dataPtr->presetProfiles[_name] = new Preset;
  this->dataPtr->presetProfiles[_name]->at("name") = _name;
}

void PresetManager::CreateProfileFromPreset(const std::string& _name,
  Preset* _preset)
{
  this->dataPtr->presetProfiles[_name] = _preset;
  this->dataPtr->presetProfiles[_name]->at("name") = _name;
}

sdf::ElementPtr PresetManager::GetSDFForProfile(const std::string &_name)
{
  return NULL;
}

void PresetManager::SetSDFForProfile(const std::string &_name)
{

}
