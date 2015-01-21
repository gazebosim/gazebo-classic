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

#include "gazebo/physics/PresetManager.hh"
#include "gazebo/physics/PresetManagerPrivate.hh"

PresetManager::PresetManager(PhysicsEngine _engine)
{

}

virtual ~PresetManager::PresetManager();

virtual void PresetManager::Load(sdf::ElementPtr _sdf);

void PresetManager::SetCurrentProfile(const std::string& _name);

Preset* PresetManager::GetCurrentProfile()
{

}

std::string PresetManager::GetCurrentProfileName();

std::vector<std::string> PresetManager::GetAllProfiles();

bool PresetManager::SetProfileParam(const std::string& _profileName, const std::string& _key, const boost::any &_value);

bool PresetManager::SetCurrentProfileParam(const std::string& _key, const boost::any &_value);

void PresetManager::CreateProfile(const std::string& _name);

void PresetManager::CreateProfileFromPreset(const std::string& _name, Preset* _preset);

sdf::ElementPtr PresetManager::GetSDFForProfile(const std::string &_name);

void PresetManager::GetSDFForProfile(const std::string &_name);


