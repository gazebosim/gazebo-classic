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

#include "gazebo/common/Console.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Preset::Preset()
    : dataPtr(new PresetPrivate)
{
}

//////////////////////////////////////////////////
Preset::Preset(const std::string &_name)
    : dataPtr(new PresetPrivate(_name))
{
}

//////////////////////////////////////////////////
Preset::~Preset()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
bool Preset::SetAllPhysicsParameters(PhysicsEnginePtr _physicsEngine) const
{
  bool result = true;
  for (auto param : this->dataPtr->parameterMap)
  {
    if (!_physicsEngine->SetParam(param.first, param.second))
    {
      result = false;
    }
  }
  return result;
}

//////////////////////////////////////////////////
std::string Preset::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Preset::Name(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
bool Preset::HasParam(const std::string &_key) const
{
  return (this->dataPtr->parameterMap.find(_key) !=
      this->dataPtr->parameterMap.end());
}

//////////////////////////////////////////////////
bool Preset::GetParam(const std::string &_key, boost::any &_value) const
{
  if (!this->HasParam(_key))
  {
    gzwarn << "Parameter " << _key << " is not a member of Preset"
           << this->Name() << std::endl;
    return false;
  }
  _value = this->dataPtr->parameterMap[_key];
  return true;
}

//////////////////////////////////////////////////
void Preset::SetParam(const std::string &_key, const boost::any &_value)
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
  // TODO: check for non-physics element in Preset::SDF (set)
  this->dataPtr->elementSDF = _sdfElement;
}

//////////////////////////////////////////////////
PresetManager::PresetManager(PhysicsEnginePtr _physicsEngine,
    sdf::ElementPtr _sdf) : dataPtr(new PresetManagerPrivate)
{
  this->dataPtr->physicsEngine = _physicsEngine;

  // Load SDF
  if (_sdf->HasElement("physics"))
  {
    bool defaultSet = false;
    for (sdf::ElementPtr physicsElem = _sdf->GetElement("physics"); physicsElem;
          physicsElem = physicsElem->GetNextElement("physics"))
    {
      // Get name attribute
      std::string name = this->CreateProfile(physicsElem);
      if (!name.empty())
      {
        if (this->CurrentProfile().empty())
        {
          this->CurrentProfile(name);
        }
        if (physicsElem->HasAttribute("default"))
        {
          if (physicsElem->Get<bool>("default") && !defaultSet)
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
  delete this->dataPtr;
}

//////////////////////////////////////////////////
bool PresetManager::CurrentProfile(const std::string &_name)
{
  if (_name == this->CurrentProfile())
    return true;

  if (_name.size() <= 0)
    return false;

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
  this->dataPtr->currentPreset = _name;
  return this->CurrentPreset()->SetAllPhysicsParameters(
      this->dataPtr->physicsEngine);
}

//////////////////////////////////////////////////
std::string PresetManager::CurrentProfile() const
{
  return this->dataPtr->currentPreset;
}

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
bool PresetManager::SetProfileParam(const std::string &_profileName,
    const std::string &_key, const boost::any &_value)
{
  if (_profileName == this->CurrentProfile())
  {
    return this->SetCurrentProfileParam(_key, _value);
  }

  if (this->dataPtr->presetProfiles.find(_profileName) ==
      this->dataPtr->presetProfiles.end())
  {
    return false;
  }
  if (!this->dataPtr->presetProfiles[_profileName].HasParam(_key))
  {
    return false;
  }
  this->dataPtr->presetProfiles[_profileName].SetParam(_key, _value);
  return true;
}

//////////////////////////////////////////////////
bool PresetManager::GetProfileParam(const std::string &_name,
    const std::string &_key, boost::any &_value) const
{
  if (this->dataPtr->presetProfiles.find(_name) ==
      this->dataPtr->presetProfiles.end())
  {
    return false;
  }
  return this->dataPtr->presetProfiles[_name].GetParam(_key, _value);
}

//////////////////////////////////////////////////
bool PresetManager::SetCurrentProfileParam(const std::string &_key,
    const boost::any &_value)
{
  if (this->CurrentProfile().empty())
  {
    return false;
  }
  if (!this->CurrentPreset()->HasParam(_key))
  {
    return false;
  }
  this->CurrentPreset()->SetParam(_key, _value);
  try
  {
    return this->dataPtr->physicsEngine->SetParam(_key, _value);
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "Couldn't set physics engine parameter! " << e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool PresetManager::GetCurrentProfileParam(const std::string &_key,
    boost::any &_value)
{
  if (!this->CurrentPreset())
  {
    return false;
  }
  return this->CurrentPreset()->GetParam(_key, _value);
}

//////////////////////////////////////////////////
void PresetManager::CreateProfile(const std::string &_name)
{
  if (_name.empty())
  {
    gzwarn << "Specified profile name was invalid. Aborting." << std::endl;
    return;
  }
  if (this->dataPtr->presetProfiles.find(_name) !=
      this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Warning: profile " << _name << " already exists! Overwriting."
           << std::endl;
  }

  this->dataPtr->presetProfiles[_name] = Preset(_name);
}

//////////////////////////////////////////////////
std::string PresetManager::CreateProfile(sdf::ElementPtr _elem)
{
  if (!_elem->HasAttribute("name"))
  {
    return "";
  }
  const std::string name = _elem->Get<std::string>("name");
  if (name.empty())
    return "";

  this->CreateProfile(name);
  this->ProfileSDF(name, _elem);
  return name;
}

//////////////////////////////////////////////////
void PresetManager::RemoveProfile(const std::string &_name)
{
  if (_name == this->CurrentProfile())
  {
    gzmsg << "deselecting current preset " << _name << std::endl;
    this->dataPtr->currentPreset = "";
  }

  this->dataPtr->presetProfiles.erase(_name);
}

//////////////////////////////////////////////////
sdf::ElementPtr PresetManager::ProfileSDF(const std::string &_name) const
{
  if (this->dataPtr->presetProfiles.find(_name) ==
      this->dataPtr->presetProfiles.end())
    return NULL;

  this->dataPtr->presetProfiles[_name].SDF(this->ProfileSDF(_name));
  return this->dataPtr->presetProfiles[_name].SDF();
}

//////////////////////////////////////////////////
void PresetManager::ProfileSDF(const std::string &_name,
    sdf::ElementPtr _sdf)
{
  if (this->dataPtr->presetProfiles.find(_name) ==
      this->dataPtr->presetProfiles.end())
    return;
  this->dataPtr->presetProfiles[_name].SDF(_sdf);

  this->GeneratePresetFromSDF(&this->dataPtr->presetProfiles[_name], _sdf);
}

//////////////////////////////////////////////////
// This sort of duplicates a code block in the constructor of SDF::Param
// (http://bit.ly/175LWfE)
boost::any GetAnySDFValue(const sdf::ElementPtr _elem)
{
  boost::any ret;

  if (typeid(int) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<int>();
  }
  else if (typeid(double) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<double>();
  }
  else if (typeid(float) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<float>();
  }
  else if (typeid(bool) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<bool>();
  }
  else if (typeid(std::string) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<std::string>();
  }
  else if (typeid(sdf::Vector3) == _elem->GetValue()->GetType())
  {
    ret = _elem->Get<gazebo::math::Vector3>();
  }
  else
  {
    gzerr << "Type of element [" << _elem->GetName() << "] not known!"
          << std::endl;
  }

  return ret;
}


//////////////////////////////////////////////////
void PresetManager::GeneratePresetFromSDF(Preset *_preset,
    const sdf::ElementPtr _elem) const
{
  if (!_preset || !_elem)
    return;
  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    if (elem->GetValue() != NULL)
    {
      _preset->SetParam(elem->GetName(), GetAnySDFValue(elem));
    }
    this->GeneratePresetFromSDF(_preset, elem);
  }
}

//////////////////////////////////////////////////
Preset *PresetManager::CurrentPreset() const
{
  return &this->dataPtr->presetProfiles[this->CurrentProfile()];
}
