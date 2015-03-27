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
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"
#include "gazebo/physics/PresetManager.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
// This sort of duplicates a code block in the constructor of SDF::Param
// (http://bit.ly/175LWfE)
boost::any GetAnySDFValue(const sdf::ElementPtr _elem)
{
  boost::any ret;

  if (!_elem)
  {
    gzerr << "got NULL ElementPtr in GetAnySDFValue" << std::endl;
    return ret;
  }

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
}

//////////////////////////////////////////////////
Preset::Preset(const std::string &_name)
    : dataPtr(new PresetPrivate(_name))
{
}

//////////////////////////////////////////////////
Preset::~Preset()
{
  GZ_ASSERT(this->dataPtr, "Data ptr not NULL for Preset!");
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
    return false;
  }

  for (auto const &param : this->dataPtr->parameterMap)
  {
    // disable params we know can't be set
    if (param.first == "type")
      continue;
    if (!_physicsEngine->SetParam(param.first, param.second))
    {
      gzwarn << "Couldn't set parameter [" << param.first
             << "] in physics engine" << std::endl;
      result = false;
    }
  }
  return result;
}

//////////////////////////////////////////////////
bool Preset::SetAllParamsFromSDF(const sdf::ElementPtr _elem)
{
  return this->SetAllParamsHelper(_elem, true);
}

//////////////////////////////////////////////////
bool Preset::SetAllParamsHelper(const sdf::ElementPtr _elem, bool _result)
{
  if (!_elem)
  {
    return _result;
  }

  // Avoid setting parameters that do not match the physics engine type set in
  // SDF

  if (_elem->GetParent() && _elem->GetParent()->GetName() == "physics" &&
      _elem->GetParent()->Get<std::string>("type") != _elem->GetName())
  {
    return _result;
  }

  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    if (elem->GetValue())
    {
      _result &= this->SetParam(elem->GetName(), GetAnySDFValue(elem));
    }
    _result &= this->SetAllParamsHelper(elem, _result);
  }
  return _result;
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
  return this->dataPtr->parameterMap.count(_key) != 0 && !_key.empty();
}

//////////////////////////////////////////////////
bool Preset::GetParam(const std::string &_key, boost::any &_value) const
{
  if (!this->HasParam(_key))
  {
    gzwarn << "Parameter [" << _key << "] is not a member of profile ["
           << this->Name() << "]" << std::endl;
    return false;
  }
  _value = this->dataPtr->parameterMap[_key];
  return true;
}

//////////////////////////////////////////////////
bool Preset::SetParam(const std::string &_key, const boost::any &_value)
{
  if (_key.empty())
    return false;
  this->dataPtr->parameterMap[_key] = _value;
  return true;
}

//////////////////////////////////////////////////
sdf::ElementPtr Preset::SDF() const
{
  return this->dataPtr->elementSDF;
}

//////////////////////////////////////////////////
void Preset::SDF(const sdf::ElementPtr _sdfElement)
{
  // TODO: check for non-physics element in Preset::SDF (set)
  GZ_ASSERT(_sdfElement, "Tried to add NULL SDF element to Preset");
  if (_sdfElement->GetName() != "physics")
  {
    gzwarn << "Can't assign non-physics element to preset profile" << std::endl;
    return;
  }
  this->dataPtr->elementSDF = _sdfElement;
}

//////////////////////////////////////////////////
PresetManager::PresetManager(PhysicsEnginePtr _physicsEngine,
    const sdf::ElementPtr _sdf)
    : dataPtr(new PresetManagerPrivate)
{
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
      continue;
    }
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

//////////////////////////////////////////////////
PresetManager::~PresetManager()
{
  GZ_ASSERT(this->dataPtr, "Data pointer not NULL for PresetManager!");
  this->dataPtr->presetProfiles.clear();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
bool PresetManager::CurrentProfile(const std::string &_name)
{
  if (_name.empty())
    return false;

  if (_name == this->CurrentProfile())
    return true;

  if (!this->HasProfile(_name))
  {
    gzwarn << "Profile [" << _name << "] not found." << std::endl;
    return false;
  }

  GZ_ASSERT(this->dataPtr->physicsEngine, "Physics engine was NULL");

  this->dataPtr->currentPreset = _name;
  if (this->CurrentPreset() == NULL)
    return false;

  // For now, ignore the return value of this function, since not all
  // parameters are supported
  this->CurrentPreset()->SetAllPhysicsParameters(
      this->dataPtr->physicsEngine);
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

  if (_profileName.empty() || !this->HasProfile(_profileName))
  {
    gzwarn << "Invalid profile name: [" << _profileName << "]" << std::endl;
    return false;
  }
  if (!this->dataPtr->presetProfiles[_profileName].HasParam(_key))
  {
    gzwarn << "Profile [" << _profileName << "] does not have key [" << _key
           << "], so it was not set." << std::endl;
    return false;
  }
  if (!this->dataPtr->presetProfiles[_profileName].SetParam(_key, _value))
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
  if (_name.empty() || !this->HasProfile(_name))
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
bool PresetManager::CreateProfile(const std::string &_name)
{
  if (_name.empty())
  {
    gzwarn << "Specified profile name was invalid. Aborting." << std::endl;
    return false;
  }
  if (this->HasProfile(_name))
  {
    gzwarn << "Warning: profile [" << _name << "] already exists! Overwriting."
           << std::endl;
  }
  Preset *newPreset = new Preset(_name);
  this->dataPtr->presetProfiles[_name] = *newPreset;

  this->ProfileSDF(_name, this->dataPtr->physicsEngine->GetSDF());

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
  return this->dataPtr->presetProfiles.count(_name) > 0;
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
void PresetManager::ProfileSDF(const std::string &_name,
    const sdf::ElementPtr _sdf)
{
  if (_name.empty() || !this->HasProfile(_name))
    return;
  this->dataPtr->presetProfiles[_name].SDF(_sdf);

  this->GeneratePresetFromSDF(&this->dataPtr->presetProfiles[_name], _sdf);

  if (_name == this->CurrentProfile())
  {
    this->CurrentPreset()->SetAllPhysicsParameters(
        this->dataPtr->physicsEngine);
  }
  this->dataPtr->presetProfiles[_name].SetAllParamsFromSDF(_sdf);
}

//////////////////////////////////////////////////
void PresetManager::GeneratePresetFromSDF(Preset *_preset,
    const sdf::ElementPtr _elem) const
{
  if (!_preset)
  {
    gzerr << "NULL preset given to GeneratePresetFromSDF. No preset will be "
          << "generated." << std::endl;
    return;
  }
  if (!_elem)
  {
    return;
  }

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
      _preset->SetParam(elem->GetName(), GetAnySDFValue(elem));
    }
    this->GeneratePresetFromSDF(_preset, elem);
  }
}

//////////////////////////////////////////////////
void PresetManager::GenerateSDFFromPreset(sdf::ElementPtr &_elem,
    const std::string &_name) const
{
  if (!this->HasProfile(_name))
  {
    gzwarn << "Profile [" << _name << "] does not exist. No SDF will be "
           << "generated." << std::endl;
    return;
  }
  _elem.reset();

  // Start with the elementSDF member variable of the preset
  _elem = this->dataPtr->presetProfiles[_name].SDF()->Clone();
  GZ_ASSERT(_elem, "Null SDF pointer in preset");

  GenerateSDFHelper(_elem, this->dataPtr->presetProfiles[_name]);
  GZ_ASSERT(_elem, "Null SDF pointer in preset");
}

//////////////////////////////////////////////////
void PresetManager::GenerateSDFHelper(sdf::ElementPtr &_elem,
    const Preset &_preset) const
{
  if (!_elem)
  {
    return;
  }

  // For each element, enforce that its equivalent in Preset has the same value
  for (sdf::ElementPtr elem = _elem->GetFirstElement(); elem;
        elem = elem->GetNextElement())
  {
    boost::any value;
    if (_preset.GetParam(elem->GetName(), value) && elem->GetValue())
    {
      // cast based on type in SDF
      if (typeid(int) == elem->GetValue()->GetType())
      {
        int v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (typeid(double) == elem->GetValue()->GetType())
      {
        double v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (typeid(float) == elem->GetValue()->GetType())
      {
        float v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (typeid(bool) == elem->GetValue()->GetType())
      {
        bool v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (typeid(std::string) == elem->GetValue()->GetType())
      {
        std::string v;
        if (CastAnyValue(value, v))
          elem->Set(v);
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
      else if (typeid(sdf::Vector3) == elem->GetValue()->GetType())
      {
        math::Vector3 v;
        if (CastAnyValue(value, v))
        {
          gzdbg << "Vector3: " << v << std::endl;
          elem->Set(v);
        }
        else
          gzerr << "SDF type did not give successful cast" << std::endl;
      }
    }
    this->GenerateSDFHelper(elem, _preset);
  }
}

//////////////////////////////////////////////////
Preset *PresetManager::CurrentPreset() const
{
  return &this->dataPtr->presetProfiles[this->CurrentProfile()];
}
