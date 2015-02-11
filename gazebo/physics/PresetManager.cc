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

//////////////////////////////////////////////////
Preset::Preset() : dataPtr(new PresetPrivate)
{
  //this->Param("name", std::string("default name"));
  this->dataPtr->name = "default name";
}

//////////////////////////////////////////////////
Preset::Preset(const std::string& _name) : dataPtr(new PresetPrivate)
{
  this->Name(_name);
}

//////////////////////////////////////////////////
std::map<std::string, boost::any> Preset::ParameterMap()
{
  return this->dataPtr->parameterMap;
}

//////////////////////////////////////////////////
std::string Preset::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Preset::Name(const std::string& _name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
boost::any Preset::Param(const std::string& _key) const
{
  if (this->dataPtr->parameterMap.find(_key) ==
      this->dataPtr->parameterMap.end())
  {
    gzwarn << "Parameter " << _key << " is not a member of Preset"
           << this->Name() << std::endl;
    return 0;
  }
  return this->dataPtr->parameterMap[_key];
}

//////////////////////////////////////////////////
void Preset::Param(const std::string& _key, const boost::any& _value)
{
  this->dataPtr->parameterMap[_key] = _value;
}

//////////////////////////////////////////////////
sdf::ElementPtr Preset::SDF() const
{
  return this->dataPtr->elementSDF;
}

//////////////////////////////////////////////////
void Preset::SDF(sdf::ElementPtr _sdfElement)
{
  this->dataPtr->elementSDF = _sdfElement;
}

//////////////////////////////////////////////////
PresetManager::PresetManager(PhysicsEnginePtr _physicsEngine, sdf::ElementPtr _sdf) :
    dataPtr(new PresetManagerPrivate)
{
  this->dataPtr->physicsEngine = _physicsEngine;
  this->dataPtr->currentPreset = NULL;

  /*std::string defaultName = "default_physics";
  if (_sdf->HasAttribute("default_physics"))
  {
    defaultName = _sdf->GetAttribute("default_physics")->GetAsString();
  }*/

  // Load SDF
  if (_sdf->HasElement("physics"))
  {
    //sdf::ElementPtr physicsElem = _sdf->GetElement("physics");
    //while (physicsElem)
    bool defaultSet = false;
    for (sdf::ElementPtr physicsElem = _sdf->GetElement("physics"); physicsElem;
          physicsElem = physicsElem->GetNextElement("physics"))
    {
      // Get our own copy of this physics element.
      sdf::ElementPtr elemCopy = physicsElem->Clone();

      // Get name attribute
      std::string name = this->CreateProfile(elemCopy);
      if (name.size() > 0)
      {
        // Put all the elements in a map

        /*if (name == defaultName)
        {
          if (!this->CurrentProfilePreset())
          {
            this->CurrentProfile(name);
          }
          else
          {
            gzwarn << "Multiple simulation presets selected in SDF. "
                   << "Ignoring preset: " << name << "." << std::endl;
          }
        }*/
        if (!this->CurrentProfilePreset())
        {
          this->CurrentProfile(name);
        }
        if (physicsElem->HasAttribute("default"))
        {
          if (!physicsElem->Get<bool>("default"))
            continue;
          if (!defaultSet)
          {
            this->CurrentProfile(name);
            defaultSet = true;
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
PresetManager::~PresetManager()
{
/*  std::vector<Preset*> allProfiles = this->GetAllProfiles();
  for (unsigned int i = 0; i < allProfiles.size(); i++)
  {
    delete allProfiles[i];
  }*/
  //delete this->dataPtr;
}

//////////////////////////////////////////////////
bool PresetManager::CurrentProfile(const std::string& _name)
{
  if (this->dataPtr->presetProfiles.find(_name) ==
      this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Profile " << _name << " not found." << std::endl;
    return false;
  }

  if (!this->dataPtr->physicsEngine)
  {
    gzwarn << "Physics engine was NULL!" << std::endl;
    return false;
  }
  Preset* futurePreset = &this->dataPtr->presetProfiles[_name];
  bool result = true;
  for (auto it = futurePreset->ParameterMap().begin();
     it != futurePreset->ParameterMap().end(); ++it)
  {
    if (!this->dataPtr->physicsEngine->SetParam(it->first, it->second))
    {
      gzerr << "Failed to set physics engine parameter" << std::endl;
      result = false;
      break;
    }
  }
  if (result)
  {
    this->dataPtr->currentPreset = &(this->dataPtr->presetProfiles[_name]);
  }
  else
  {
    // TODO: Set back all the changes in the physics engine
  }
  return result;
}

//////////////////////////////////////////////////
std::string PresetManager::CurrentProfile() const
{
  if (!this->dataPtr->currentPreset)
  {
    gzwarn << "No current preset." << std::endl;
    return "";
  }
  return this->dataPtr->currentPreset->Name();

  /*if (this->dataPtr->currentPreset->paramMap.find("name") ==
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
    gzwarn << "Got bad type in CurrentProfile" << std::endl;
    return "";
  }*/
}

//////////////////////////////////////////////////
/*std::vector<Preset*> PresetManager::AllProfiles() const
{
  std::vector<Preset*> ret;
  for (auto it = this->dataPtr->presetProfiles.begin();
     it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(&(it->second));
  }
  return ret;
}*/

//////////////////////////////////////////////////
std::vector<std::string> PresetManager::AllProfiles() const
{
  std::vector<std::string> ret;
  for (auto it = this->dataPtr->presetProfiles.begin();
         it != this->dataPtr->presetProfiles.end(); ++it)
  {
    ret.push_back(it->first);
  }
  return ret;
}

//////////////////////////////////////////////////
bool PresetManager::ProfileParam(const std::string& _profileName,
    const std::string& _key, const boost::any &_value)
{
  if (_profileName == this->CurrentProfile())
  {
    return this->CurrentProfileParam(_key, _value);
  }

  if (this->dataPtr->presetProfiles.find(_profileName) ==
      this->dataPtr->presetProfiles.end())
  {
    return false;
  }
  this->dataPtr->presetProfiles[_profileName].Param(_key, _value);
  return true;
}

//////////////////////////////////////////////////
boost::any PresetManager::ProfileParam(const std::string &_name,
    const std::string& _key) const
{
  return this->dataPtr->presetProfiles[_name].Param(_key);
}

//////////////////////////////////////////////////
bool PresetManager::CurrentProfileParam(const std::string& _key,
    const boost::any &_value)
{
  if (this->dataPtr->currentPreset == NULL)
  {
    return false;
  }
  this->dataPtr->currentPreset->Param(_key, _value);
  try
  {
    return this->dataPtr->physicsEngine->SetParam(_key, _value);
  }
  catch (const boost::bad_any_cast& e)
  {
    gzerr << "Couldn't set physics engine parameter! " << e.what() << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void PresetManager::CreateProfile(const std::string& _name)
{
  if (this->dataPtr->presetProfiles.find(_name) !=
      this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Warning: profile " << _name << " already exists! Overwriting."
           << std::endl;
  }
  else
  {
    this->dataPtr->presetProfiles.emplace(_name, Preset(_name));
  }

  /*this->dataPtr->presetProfiles[_name].Name(_name);*/
}

//////////////////////////////////////////////////
std::string PresetManager::CreateProfile(sdf::ElementPtr _elem)
{
  if (!_elem->HasAttribute("name"))
  {
    return "";
  }
  const std::string name = _elem->Get<std::string>("name");

  this->CreateProfile(name);
  this->ProfileSDF(name, _elem);
  this->dataPtr->presetProfiles[name].Name(name);
  return name;
}

//////////////////////////////////////////////////
void PresetManager::RemoveProfile(const std::string& _name)
{
  if (_name == this->CurrentProfile())
  {
    gzwarn << "deselecting current preset " << _name << std::endl;
    this->dataPtr->currentPreset = NULL;
  }

  this->dataPtr->presetProfiles.erase(_name);
  // this->dataPtr->presetSDF.erase(_name);
}

//////////////////////////////////////////////////
sdf::ElementPtr PresetManager::ProfileSDF(const std::string &_name) const
{
  /*this->dataPtr->presetSDF[_name] =
      this->GenerateSDFFromPreset(this->dataPtr->presetProfiles[_name]);
  return this->dataPtr->presetSDF[_name];*/
  this->dataPtr->presetProfiles[_name].SDF(this->ProfileSDF(_name));
  return this->dataPtr->presetProfiles[_name].SDF();
}

//////////////////////////////////////////////////
void PresetManager::ProfileSDF(const std::string &_name,
  sdf::ElementPtr _sdf)
{
  //this->dataPtr->presetSDF[_name] = _sdf;
  this->dataPtr->presetProfiles[_name].SDF(_sdf);

  this->dataPtr->presetProfiles[_name] = this->GeneratePresetFromSDF(_sdf);
}

//////////////////////////////////////////////////
Preset PresetManager::GeneratePresetFromSDF(const sdf::ElementPtr _elem) const
{
  Preset preset; 

  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    std::cout << "Setting element " << elem->GetName() << " as: " << elem->Get<std::string>() << std::endl;
    preset.Param(elem->GetName(), elem->Get<std::string>());
  }
  preset.SDF(_elem);
  return preset;
}

//////////////////////////////////////////////////
sdf::ElementPtr PresetManager::GenerateSDFFromPreset(Preset* _preset) const
{
  sdf::ElementPtr elem(new sdf::Element);
  elem->SetName("physics");
  elem->AddAttribute("name", "string", "", true);
  sdf::ParamPtr nameParam = elem->GetAttribute("name");
  if (_preset->Name().length()==0)
  {
    gzwarn << "Name not set in parameter map, returning from GenerateSDFFromPreset!" << std::endl;
    return elem;
  }

  nameParam->Set(_preset->Name());

  for (auto &param : _preset->ParameterMap())
  {
    std::string key = param.first;
    std::string value;
    try
    {
      value = boost::any_cast<std::string>(param.second);
    }
    catch (const boost::bad_any_cast& e)
    {
      gzwarn << "Bad cast of boost::any in GenerateSDFFromPreset" << std::endl;
      return elem;
    }
    sdf::ElementPtr child = elem->AddElement(key);
    child->Set<std::string>(value);
  }
  _preset->SDF(elem);
  return elem;
}

//////////////////////////////////////////////////
Preset* PresetManager::CurrentProfilePreset() const
{
  return this->dataPtr->currentPreset;
}
