/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
template<typename T> bool CastAnyValue(const boost::any &_value, T &_return)
{
  try
  {
    _return = boost::any_cast<T>(_value);
  }
  catch(boost::bad_any_cast &_e)
  {
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
Preset::Preset()
    : dataPtr(new PresetPrivate)
{
  GZ_ASSERT(this->dataPtr != NULL, "Data ptr NULL for Preset!");
}

//////////////////////////////////////////////////
Preset::Preset(const std::string &_name)
    : dataPtr(new PresetPrivate(_name))
{
  GZ_ASSERT(this->dataPtr != NULL, "Data ptr NULL for Preset!");
}

//////////////////////////////////////////////////
Preset::~Preset()
{
  this->dataPtr->elementSDF.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
bool Preset::SetAllPhysicsParameters(PhysicsEnginePtr _physicsEngine) const
{
  bool result = true;

  if (!_physicsEngine)
  {
    gzwarn << "Physics engine for PresetManager is NULL. PresetManager will "
           << "have no effect on simulation!" << std::endl;
    result = false;
  }
  else
  {
    for (auto const &param : this->dataPtr->parameterMap)
    {
      // disable params we know can't be set
      if (param.first != "type" &&
          !_physicsEngine->SetParam(param.first, param.second))
      {
        gzwarn << "Couldn't set parameter [" << param.first
          << "] in physics engine" << std::endl;
        result = false;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool Preset::SetAllParamsFromSDF(const sdf::ElementPtr _elem)
{
  return this->SetAllParamsHelper(_elem);
}

//////////////////////////////////////////////////
bool Preset::SetAllParamsHelper(const sdf::ElementPtr _elem)
{
  bool result = true;

  if (!_elem)
  {
    return result;
  }

  // Avoid setting parameters that do not match the physics engine type set in
  // SDF
  if (_elem->GetParent() && _elem->GetParent()->GetName() == "physics" &&
      _elem->GetParent()->Get<std::string>("type") != _elem->GetName())
  {
    return result;
  }

  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    if (elem->GetValue())
    {
      result &= this->SetParam(elem->GetName(), elem->GetAny());
    }
    result &= this->SetAllParamsHelper(elem);
  }

  return result;
}

//////////////////////////////////////////////////
std::string Preset::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
bool Preset::HasParam(const std::string &_key) const
{
  return this->dataPtr->parameterMap.find(_key) !=
         this->dataPtr->parameterMap.end();
}

//////////////////////////////////////////////////
bool Preset::GetParam(const std::string &_key, boost::any &_value) const
{
  bool result = true;

  if (!this->HasParam(_key))
  {
    gzwarn << "Parameter [" << _key << "] is not a member of profile ["
           << this->Name() << "]" << std::endl;
    result = false;
  }
  else
  {
    _value = this->dataPtr->parameterMap[_key];
  }

  return result;
}

//////////////////////////////////////////////////
bool Preset::SetParam(const std::string &_key, const boost::any &_value)
{
  bool result = true;

  if (_key.empty())
    result = false;
  else
    this->dataPtr->parameterMap[_key] = _value;

  return result;
}

//////////////////////////////////////////////////
sdf::ElementPtr Preset::SDF() const
{
  return this->dataPtr->elementSDF;
}

//////////////////////////////////////////////////
bool Preset::SDF(const sdf::ElementPtr _sdfElement)
{
  if (!_sdfElement)
  {
    gzwarn << "Can't add NULL SDF element to Preset" << std::endl;
    return false;
  }
  if (_sdfElement->GetName() != "physics")
  {
    gzwarn << "Can't assign non-physics element to preset profile" << std::endl;
    return false;
  }
  this->dataPtr->elementSDF = _sdfElement;
  return true;
}

//////////////////////////////////////////////////
PresetManager::PresetManager(PhysicsEnginePtr _physicsEngine,
    const sdf::ElementPtr _sdf)
    : dataPtr(new PresetManagerPrivate)
{
  GZ_ASSERT(this->dataPtr != NULL, "Data pointer NULL for PresetManager!");

  this->dataPtr->physicsEngine = _physicsEngine;
  if (!_physicsEngine || !_sdf)
  {
    gzerr << "Required arguments NULL for PresetManager" << std::endl;
    return;
  }

  // Load SDF
  if (!_sdf->HasElement("physics"))
  {
    gzerr << "PresetManager does not support preset profiles for SDF besides "
          << "physics. No profiles will be made." << std::endl;
  }

  bool defaultSet = false;
  for (sdf::ElementPtr physicsElem = _sdf->GetElement("physics"); physicsElem;
        physicsElem = physicsElem->GetNextElement("physics"))
  {
    // Get name attribute
    std::string name = this->CreateProfile(physicsElem);
    if (name.empty())
    {
      gzlog << "Empty physics profile name specified in SDF. No profile made "
            << "in PresetManager." << std::endl;
    }
    else
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

//////////////////////////////////////////////////
PresetManager::~PresetManager()
{
  this->dataPtr->physicsEngine.reset();
  this->dataPtr->presetProfiles.clear();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
bool PresetManager::CurrentProfile(const std::string &_name)
{
  GZ_ASSERT(this->dataPtr->physicsEngine, "Physics engine was NULL");

  if (_name.empty())
    return false;

  if (!this->HasProfile(_name))
  {
    gzwarn << "Profile [" << _name << "] not found." << std::endl;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->currentProfileMutex);

    if (_name == this->CurrentProfile())
      return true;

    this->dataPtr->currentPreset = _name;

    // For now, ignore the return value of this function, since not all
    // parameters are supported
    this->CurrentPreset()->SetAllPhysicsParameters(
        this->dataPtr->physicsEngine);
  }

  return true;
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
  for (auto const &profile : this->dataPtr->presetProfiles)
  {
    ret.push_back(profile.first);
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

  auto iter = this->dataPtr->presetProfiles.find(_profileName);

  if (iter == this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Invalid profile name: [" << _profileName << "]" << std::endl;
    return false;
  }

  if (!iter->second.HasParam(_key))
  {
    gzwarn << "Profile [" << _profileName << "] does not have key [" << _key
           << "], so it was not set." << std::endl;
    return false;
  }

  if (!iter->second.SetParam(_key, _value))
  {
    gzwarn << "Could not set key [" << _key
           << "] for profile [" << _profileName << "]." << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool PresetManager::GetProfileParam(const std::string &_name,
    const std::string &_key, boost::any &_value) const
{
  if (!this->HasProfile(_name))
  {
    return false;
  }
  return this->dataPtr->presetProfiles[_name].GetParam(_key, _value);
}

//////////////////////////////////////////////////
bool PresetManager::SetCurrentProfileParam(const std::string &_key,
    const boost::any &_value)
{
  if (this->CurrentProfile().empty() || this->CurrentPreset() == NULL)
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->currentProfileMutex);
  if (!this->CurrentPreset()->HasParam(_key))
  {
    return false;
  }
  if (!this->CurrentPreset()->SetParam(_key, _value))
    return false;
  try
  {
    return this->dataPtr->physicsEngine->SetParam(_key, _value);
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "Couldn't set physics engine parameter[" << e.what()
          << "]" << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
bool PresetManager::GetCurrentProfileParam(const std::string &_key,
    boost::any &_value)
{
  if (this->CurrentProfile().empty())
  {
    return false;
  }
  return this->CurrentPreset()->GetParam(_key, _value);
}

//////////////////////////////////////////////////
bool PresetManager::CreateProfile(const std::string &_name)
{
  if (_name.empty())
  {
    gzwarn << "Specified profile name was empty. Aborting." << std::endl;
    return false;
  }
  if (this->HasProfile(_name))
  {
    gzwarn << "Warning: profile [" << _name << "] already exists! Overwriting."
           << std::endl;
  }
  this->dataPtr->presetProfiles.emplace(_name, _name);

  if (!this->ProfileSDF(_name, this->dataPtr->physicsEngine->GetSDF()))
    return false;

  return true;
}

//////////////////////////////////////////////////
std::string PresetManager::CreateProfile(const sdf::ElementPtr _elem)
{
  if (!_elem || !_elem->HasAttribute("name"))
  {
    return "";
  }
  const std::string name = _elem->Get<std::string>("name");
  if (name.empty())
    return "";

  if (!this->CreateProfile(name))
    return "";

  // Make a copy of this SDF element.
  this->ProfileSDF(name, _elem);
  return name;
}

//////////////////////////////////////////////////
void PresetManager::RemoveProfile(const std::string &_name)
{
  if (!this->HasProfile(_name))
  {
    gzwarn << "Cannot remove non-existent profile [" << _name << "]"
           << std::endl;
    return;
  }

  if (_name == this->CurrentProfile())
  {
    gzmsg << "deselecting current preset " << _name << std::endl;
    this->dataPtr->currentPreset = "";
  }

  this->dataPtr->presetProfiles.erase(_name);
}

//////////////////////////////////////////////////
bool PresetManager::HasProfile(const std::string &_name) const
{
  return this->dataPtr->presetProfiles.find(_name) !=
         this->dataPtr->presetProfiles.end();
}

//////////////////////////////////////////////////
sdf::ElementPtr PresetManager::ProfileSDF(const std::string &_name) const
{
  if (_name.empty() || !this->HasProfile(_name))
  {
    gzerr << "Profile [" << _name << "] does not exist. Returning NULL "
          << "SDF pointer." << std::endl;
    return NULL;
  }

  return this->dataPtr->presetProfiles[_name].SDF();
}

//////////////////////////////////////////////////
bool PresetManager::ProfileSDF(const std::string &_name,
    const sdf::ElementPtr _sdf)
{
  if (!_sdf)
  {
    gzwarn << "Received NULL SDF element pointer in ProfileSDF" << std::endl;
    return false;
  }

  auto iter = this->dataPtr->presetProfiles.find(_name);

  if (iter == this->dataPtr->presetProfiles.end() || !iter->second.SDF(_sdf))
    return false;

  this->GeneratePresetFromSDF(_sdf, iter->second);

  if (_name == this->CurrentProfile())
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->currentProfileMutex);
    this->CurrentPreset()->SetAllPhysicsParameters(
        this->dataPtr->physicsEngine);
  }

  iter->second.SetAllParamsFromSDF(_sdf);
  return true;
}

//////////////////////////////////////////////////
void PresetManager::GeneratePresetFromSDF(const sdf::ElementPtr _elem,
    Preset &_preset) const
{
  GZ_ASSERT(_elem, "NULL elem in GeneratePresetFromSDF, should never happen");

  // Check for physics engine type. If the type specified in SDF doesn't match
  // the type of the physics engine, don't add the children to the parameter
  // map. This will be changed in the future when switching the type of the
  // physics engine is allowed.
  GZ_ASSERT(this->dataPtr->physicsEngine, "No physics engine in PresetManager");
  if (_elem->GetParent() && _elem->GetParent()->GetName() == "physics" &&
      this->dataPtr->physicsEngine->GetType() != _elem->GetName())
  {
    return;
  }

  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    if (elem->GetValue() != NULL)
    {
      _preset.SetParam(elem->GetName(), elem->GetAny());
    }
    this->GeneratePresetFromSDF(elem, _preset);
  }
}

//////////////////////////////////////////////////
void PresetManager::GenerateSDFFromPreset(const std::string &_name,
    sdf::ElementPtr &_elem) const
{
  auto iter = this->dataPtr->presetProfiles.find(_name);

  if (iter == this->dataPtr->presetProfiles.end())
  {
    gzwarn << "Profile [" << _name << "] does not exist. No SDF will be "
           << "generated." << std::endl;
    return;
  }
  _elem.reset();

  // Start with the elementSDF member variable of the preset
  _elem = iter->second.SDF()->Clone();
  GZ_ASSERT(_elem, "Null SDF pointer in preset");

  this->GenerateSDFHelper(iter->second, _elem);
  GZ_ASSERT(_elem, "Generated NULL SDF pointer");
}

//////////////////////////////////////////////////
void PresetManager::GenerateSDFHelper(const Preset &_preset,
    sdf::ElementPtr &_elem) const
{
  GZ_ASSERT(_elem, "NULL elem in GenerateSDFHelper, should never happen");

  // For each element, enforce that its equivalent in Preset has the same value
  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    boost::any value;
    if (_preset.GetParam(elem->GetName(), value) && elem->GetValue())
    {
      // cast based on type in SDF
      if (elem->GetValue()->IsType<int>())
      {
        int v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (elem->GetValue()->IsType<double>())
      {
        double v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (elem->GetValue()->IsType<float>())
      {
        float v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (elem->GetValue()->IsType<bool>())
      {
        bool v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (elem->GetValue()->IsType<std::string>())
      {
        std::string v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (elem->GetValue()->IsType<ignition::math::Vector3d>())
      {
        ignition::math::Vector3d v;
        if (CastAnyValue(value, v))
        {
          gzdbg << "Vector3: " << v << std::endl;
          elem->Set(v);
        }
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      }
      else if (elem->GetValue()->IsType<sdf::Vector3>())
      {
        math::Vector3 v;
        if (CastAnyValue(value, v))
        {
          gzdbg << "Vector3: " << v << std::endl;
          elem->Set(v);
        }
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
      }
    }
    this->GenerateSDFHelper(_preset, elem);
  }
}

//////////////////////////////////////////////////
Preset *PresetManager::CurrentPreset() const
{
  return &this->dataPtr->presetProfiles[this->CurrentProfile()];
}
