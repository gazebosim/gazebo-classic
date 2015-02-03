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
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

Preset::

////////////////////////////////////////////////////////////////////////////////
Preset PresetManager::GeneratePresetFromSDF(const sdf::ElementPtr _elem) const
{
  Preset preset; 

  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    gzdbg << "Getting element" << std::endl;
    preset.paramMap[elem->GetName()] = elem->Get<std::string>();
  }
  preset.sdfElement = _elem;
  return preset;
}

////////////////////////////////////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GenerateSDFFromPreset(Preset* _preset) const
{
  sdf::ElementPtr elem(new sdf::Element);
  elem->SetName("physics");
  elem->AddAttribute("name", "string", "", true);
  sdf::ParamPtr nameParam = elem->GetAttribute("name");
  if (_preset->paramMap.find("name") == _preset->paramMap.end())
  {
    gzwarn << "Name not set in parameter map, returning from GenerateSDFFromPreset!" << std::endl;
    return elem;
  }
  try
  {
    nameParam->Set(boost::any_cast<std::string>(_preset->paramMap["name"]));
  }
  catch (boost::bad_any_cast)
  {
    gzwarn << "Bad cast for name in GenerateSDFFromPreset" << std::endl;
  }
  for (auto &param : _preset->paramMap)
  {
    std::string key = param.first;
    std::string value = boost::any_cast<std::string>(param.second);
    sdf::ElementPtr child = elem->AddElement(key);
    child->Set<std::string>(value);
  }
  _preset->sdfElement = elem;
  return elem;
}

////////////////////////////////////////////////////////////////////////////////
PresetManager::PresetManager(PhysicsEnginePtr _physicsEngine, sdf::ElementPtr _sdf) :
    dataPtr(new PresetManagerPrivate)
{
  this->dataPtr->physicsEngine = _physicsEngine;
  this->dataPtr->currentPreset = NULL;

  std::string defaultName = "default_physics";
  if (_sdf->HasAttribute("default_physics"))
  {
    defaultName = _sdf->GetAttribute("default_physics")->GetAsString();
  }

  // Load SDF
  if (_sdf->HasElement("physics"))
  {
    //sdf::ElementPtr physicsElem = _sdf->GetElement("physics");
    //while (physicsElem)
    for (sdf::ElementPtr physicsElem = _sdf->GetElement("physics"); physicsElem;
          physicsElem = physicsElem->GetNextElement("physics"))
    {
      // Get our own copy of this physics element.
      sdf::ElementPtr elemCopy = physicsElem->Clone();

      gzdbg << "got physics elem " << std::endl;
      // Get name attribute
      std::string name = this->CreateProfile(elemCopy);
      if (name.size() > 0)
      {
        gzdbg << "Created physics profile " << name << std::endl;
        // Put all the elements in a map

        if (name == defaultName)
        {
          if (!this->GetCurrentProfilePreset())
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
    }
  }
  gzdbg << "finished constructing PresetManager" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
PresetManager::~PresetManager()
{
/*  std::vector<Preset*> allProfiles = this->GetAllProfiles();
  for (unsigned int i = 0; i < allProfiles.size(); i++)
  {
    delete allProfiles[i];
  }*/
  //delete this->dataPtr;
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
  this->dataPtr->currentPreset = &(this->dataPtr->presetProfiles[_name]);

  bool result = true;
  for (auto it = this->dataPtr->currentPreset->paramMap.begin();
     it != this->dataPtr->currentPreset->paramMap.end(); ++it)
  {
    result = result &&
      this->dataPtr->physicsEngine->SetParam(it->first, it->second);
  }
  return result;
}

////////////////////////////////////////////////////////////////////////////////
Preset* PresetManager::GetCurrentProfilePreset() const
{
  return this->dataPtr->currentPreset;
}

////////////////////////////////////////////////////////////////////////////////
std::string PresetManager::GetCurrentProfile() const
{
  if (!this->dataPtr->currentPreset)
  {
    gzwarn << "No current preset." << std::endl;
    return "";
  }

  if (this->dataPtr->currentPreset->paramMap.find("name") ==
      this->dataPtr->currentPreset->paramMap.end())
  {
    gzwarn << "Current preset does not have name" << std::endl;
    return "";
  }

  try
  {
    return boost::any_cast<std::string>(
      this->dataPtr->currentPreset->paramMap["name"]);
  }
  catch (boost::bad_any_cast)
  {
    gzwarn << "Got bad type in GetCurrentProfile" << std::endl;
    return "";
  }
}

////////////////////////////////////////////////////////////////////////////////
std::vector<Preset*> PresetManager::GetAllProfiles() const
{
  std::vector<Preset*> ret;
  for (auto it = this->dataPtr->presetProfiles.begin();
     it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(&(it->second));
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> PresetManager::GetAllProfiles() const
{
  std::vector<std::string> ret;
  for (auto it = this->dataPtr->presetProfiles.begin();
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
  if (_profileName == this->GetCurrentProfile())
  {
    return this->SetCurrentProfileParam(_key, _value);
  }


  this->dataPtr->presetProfiles[_profileName].paramMap[_key] = _value;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool PresetManager::SetCurrentProfileParam(const std::string& _key,
  const boost::any &_value)
{
  if (this->dataPtr->currentPreset == NULL)
  {
    return false;
  }
  this->dataPtr->currentPreset->paramMap[_key] = _value;
  return this->dataPtr->physicsEngine->SetParam(_key, _value);
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::CreateProfile(const std::string& _name)
{
  // TODO: Check if profile exists
  if (this->dataPtr->presetProfiles.find(_name) !=
      this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Warning: profile " << _name << " already exists! Overwriting."
           << std::endl;
  }
  else
  {
    this->dataPtr->presetProfiles.emplace(_name, Preset());
  }

  this->dataPtr->presetProfiles[_name].paramMap["name"] = _name;
}

////////////////////////////////////////////////////////////////////////////////
std::string PresetManager::CreateProfile(sdf::ElementPtr _elem)
{
  if (!_elem->HasAttribute("name"))
  {
    return "";
  }
  const std::string name = _elem->GetAttribute("name")->GetAsString();

  this->CreateProfile(name);
  this->SetProfileSDF(name, _elem);
  this->dataPtr->presetProfiles[name].paramMap["name"] = name;
  return name;
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::RemoveProfile(const std::string& _name)
{
  if (_name == this->GetCurrentProfile())
  {
    gzwarn << "deselecting current preset " << _name << std::endl;
    this->dataPtr->currentPreset = NULL;
  }

  this->dataPtr->presetProfiles.erase(_name);
  // this->dataPtr->presetSDF.erase(_name);
}

////////////////////////////////////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GetProfileSDF(const std::string &_name) const
{
  /*this->dataPtr->presetSDF[_name] =
      this->GenerateSDFFromPreset(this->dataPtr->presetProfiles[_name]);
  return this->dataPtr->presetSDF[_name];*/
  this->dataPtr->presetProfiles[_name].sdfElement = this->GetProfileSDF(_name);
  return this->dataPtr->presetProfiles[_name].sdfElement;
}

////////////////////////////////////////////////////////////////////////////////
void PresetManager::SetProfileSDF(const std::string &_name,
  sdf::ElementPtr _sdf)
{
  //this->dataPtr->presetSDF[_name] = _sdf;
  this->dataPtr->presetProfiles[_name].sdfElement = _sdf;

  this->dataPtr->presetProfiles[_name] = this->GeneratePresetFromSDF(_sdf);
}
