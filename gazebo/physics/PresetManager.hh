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

#ifndef _PRESETMANAGER_HH_
#define _PRESETMANAGER_HH_

#include <string>
#include <vector>

#include "gazebo/physics/PhysicsEngine.hh"

namespace gazebo
{
  namespace physics
  {
    class PresetPrivate;

    class Preset
    {
      public: Preset();
      public: Preset(const std::string & _name);
      public: std::string Name() const;
      public: void Name(const std::string& _name);
      public: boost::any Param(const std::string& _key) const;
      public: void Param(const std::string& _key, const boost::any& _value);
      public: std::map<std::string, boost::any> ParameterMap();
      public: sdf::ElementPtr SDF() const;
      public: void SDF(sdf::ElementPtr _sdfElement);
      private: PresetPrivate *dataPtr;
    };

    class PresetManagerPrivate;

    class GAZEBO_VISIBLE PresetManager
    {
      public: PresetManager(PhysicsEnginePtr _physicsEngine,
          sdf::ElementPtr _sdf);

      public: ~PresetManager();

      public: bool CurrentProfile(const std::string& _name);

      public: std::string CurrentProfile() const;

      public: std::vector<std::string> AllProfiles() const;

      public: bool ProfileParam(const std::string& _profileName,
                                   const std::string& _key,
                                   const boost::any &_value);

      public: boost::any ProfileParam(const std::string &_name,
          const std::string& _key) const;

      public: bool CurrentProfileParam(const std::string& _key,
                                          const boost::any &_value);

      public: void CreateProfile(const std::string& _name);

      public: std::string CreateProfile(sdf::ElementPtr _sdf);

      public: void RemoveProfile(const std::string& _name);

      public: sdf::ElementPtr ProfileSDF(const std::string &_name) const;

      public: void ProfileSDF(const std::string &_name, sdf::ElementPtr _sdf);

      private: Preset GeneratePresetFromSDF(const sdf::ElementPtr _elem) const;

      private: sdf::ElementPtr GenerateSDFFromPreset(Preset* _paramMap) const;

      private: Preset* CurrentProfilePreset() const;

      private: PresetManagerPrivate *dataPtr;
    };

  }  // namespace physics
}  //namespace gazebo

#endif
