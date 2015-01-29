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

////////////////////////////////////////////////////////////////////////////////
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
    gzdbg << "Getting element" << std::endl;
    (*_paramMap)[elem->GetName()] = elem->Get<std::string>();
    elem = elem->GetNextElement();
  }
}

////////////////////////////////////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GetSDFFromPreset(Preset* _paramMap) const
{
  sdf::ElementPtr elem(new sdf::Element);
  elem->SetName("physics");
  elem->AddAttribute("name", "string", "", true);
  sdf::ParamPtr name = elem->GetAttribute("name");
  name->Set(boost::any_cast<std::string>((*_paramMap)["name"]));
  for (auto &param : *_paramMap)
  {
    std::string key = param.first;
    std::string value = boost::any_cast<std::string>(param.second);
    sdf::ElementPtr child = elem->AddElement(key);
    child->Set<std::string>(value);
  }
  return elem;
}

////////////////////////////////////////////////////////////////////////////////
PresetManager::PresetManager(WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->dataPtr->physicsEngine = _world->GetPhysicsEngine();

  std::string defaultName;
  if (_sdf->HasAttribute("default_physics"))
  {
    defaultName = _sdf->GetAttribute("default_physics")->GetAsString();
  }

  // Load SDF
  if (_sdf->HasElement("physics"))
  {
    sdf::ElementPtr physicsElem = _sdf->GetElement("physics");
    while (physicsElem)
    {
      gzdbg << "got physics elem " << std::endl;
      // Get name attribute
      if (physicsElem->HasAttribute("name"))
      {
        const std::string name = physicsElem->GetAttribute("name")->GetAsString();

        gzdbg << "Creating physics profile " << name << std::endl;
        // Put all the elements in a map
        Preset *paramMap = new Preset;

        this->SetPresetFromSDF(physicsElem, paramMap);

        this->CreateProfileFromPreset(name, paramMap);
        this->dataPtr->presetSDF[name] = physicsElem;

        /*if (name == defaultName)
        {
          if (!this->GetCurrentProfile())
          {
            this->SetCurrentProfile(name);
          }
          else
          {
            gzwarn << "Multiple simulation presets selected in SDF. "
                   << "Ignoring preset: " << name << "." << std::endl;
          }
        }*/
      }
      physicsElem = physicsElem->GetNextElement("physics");
    }
  }
  gzdbg << "finished constructing PresetManager" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
PresetManager::~PresetManager()
{
  std::vector<Preset*> allProfiles = this->GetAllProfiles();
  for (unsigned int i = 0; i < allProfiles.size(); i++)
  {
    delete allProfiles[i];
  }
  delete this->dataPtr;
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
Preset* PresetManager::GetCurrentProfile() const
{
  return this->dataPtr->currentPreset;
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
void PresetManager::CreateProfile(const std::string& _name)
{
  // TODO: Check if profile exists
  this->dataPtr->presetProfiles[_name] = new Preset;
  this->dataPtr->presetProfiles[_name]->at("name") = _name;
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::CreateProfileFromPreset(const std::string& _name,
  Preset* _preset)
{
  // TODO: Check if profile exists
  if (!_preset)
    return;

  gzdbg << "create profile from preset: " << _name << std::endl;

  // accessing these members bugs out
  this->dataPtr->presetProfiles[_name] = _preset;
  this->dataPtr->presetProfiles[_name]->at("name") = _name;
  gzdbg << "created profile" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::CreateProfileFromSDF(sdf::ElementPtr _elem)
{
  Preset* preset = new Preset;
  this->SetPresetFromSDF(_elem, preset);
  this->CreateProfileFromPreset(boost::any_cast<std::string>((*preset)["name"]),
      preset);
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::RemoveProfile(const std::string& _name)
{
  if (_name == this->GetCurrentProfileName())
  {
    gzwarn << "deselecting current preset " << _name << std::endl;
    this->dataPtr->currentPreset = NULL;
  }

  this->dataPtr->presetProfiles.erase(_name);
  this->dataPtr->presetSDF.erase(_name);
}

////////////////////////////////////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GetSDFForProfile(const std::string &_name) const
{
  this->dataPtr->presetSDF[_name] =
      this->GetSDFFromPreset(this->dataPtr->presetProfiles[_name]);
  return this->dataPtr->presetSDF[_name];
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::SetSDFForProfile(const std::string &_name,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->presetSDF[_name] = _sdf;

  this->SetPresetFromSDF(_sdf, this->dataPtr->presetProfiles[_name]);
}
