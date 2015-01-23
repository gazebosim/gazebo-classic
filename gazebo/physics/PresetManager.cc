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

// TODO: mutexes

#include <boost/any.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
void PresetManager::SetPresetFromSDF(sdf::ElementPtr _preset, Preset* _paramMap)
{
  if (!_paramMap)
  {
    gzwarn << "Will not initialize NULL preset from SDF. Abort." << std::endl;
    return;
  }

  sdf::ElementPtr elem = _preset->GetFirstElement();
  while (elem)
  {
    _paramMap->at(elem->GetName()) = elem->Get<std::string>();
    elem = elem->GetNextElement();
  }
}

//////////////////////////////////////////////////
PresetManager::PresetManager(WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->dataPtr->physicsEngine = _world->GetPhysicsEngine();

  // Load SDF
  if (_sdf->HasElement("presets"))
  {
    sdf::ElementPtr preset = _sdf->GetElement("presets");
    while (preset)
    {
      // Get name attribute
      if (preset->HasAttribute("name"))
      {
        const std::string name = preset->GetAttribute("name")->GetAsString();

        // Put all the elements in a map
        Preset *paramMap = new Preset();

        this->SetPresetFromSDF(preset, paramMap);

        this->CreateProfileFromPreset(name, paramMap);
        this->dataPtr->presetSDF[name] = preset;

        bool selected = false;
        if (preset->HasAttribute("selected"))
        {
          preset->GetAttribute("selected")->Get<bool>(selected);
          if (!this->GetCurrentProfile())
          {
            this->SetCurrentProfile(name);
          }
          else
          {
            gzwarn << "Multiple simulation presets selected in SDF. "
                   << "Ignoring preset: " << name << "." << std::endl;
          }
        }
      }

      preset = preset->GetNextElement("presets");
    }
  }
}

//////////////////////////////////////////////////
PresetManager::~PresetManager()
{
  std::vector<Preset*> allProfiles = this->GetAllProfiles();
  for (unsigned int i = 0; i < allProfiles.size(); i++)
  {
    delete allProfiles[i];
  }
  delete this->dataPtr;
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
Preset* PresetManager::GetCurrentProfile() const
{
  return this->dataPtr->currentPreset;
}

//////////////////////////////////////////////////
std::string PresetManager::GetCurrentProfileName() const
{
  if (!this->dataPtr->currentPreset)
  {
    return "";
  }
  try
  {
  return boost::any_cast<std::string>(
    this->dataPtr->currentPreset->at("name"));
  }
  catch (boost::bad_any_cast)
  {
    gzwarn << "Got bad type in GetCurrentProfileName" << std::endl;
    return "";
  }
}

//////////////////////////////////////////////////
std::vector<Preset*> PresetManager::GetAllProfiles() const
{
  std::vector<Preset*> ret;
  for (std::map<std::string, Preset*>::iterator it = this->dataPtr->presetProfiles.begin();
     it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(it->second);
  }
  return ret;
}

//////////////////////////////////////////////////
std::vector<std::string> PresetManager::GetAllProfileNames() const
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

//////////////////////////////////////////////////
bool PresetManager::SetProfileParam(const std::string& _profileName,
  const std::string& _key, const boost::any &_value)
{
  if (_profileName == this->GetCurrentProfileName())
  {
    return this->SetCurrentProfileParam(_key, _value);
  }

  if (!this->dataPtr->presetProfiles[_profileName])
    return false;

  this->dataPtr->presetProfiles[_profileName]->at(_key) = _value;
  return true;
}

//////////////////////////////////////////////////
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
  if (!_preset)
    return;

  this->dataPtr->presetProfiles[_name] = _preset;
  this->dataPtr->presetProfiles[_name]->at("name") = _name;
}

//////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GetSDFForProfile(const std::string &_name) const
{
  return this->dataPtr->presetSDF[_name];
}

//////////////////////////////////////////////////
void PresetManager::SetSDFForProfile(const std::string &_name,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->presetSDF[_name] = _sdf;

  this->SetPresetFromSDF(_sdf, this->dataPtr->presetProfiles[_name]);
}
