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
    typedef std::map<std::string, boost::any> Preset;

    class PresetManagerPrivate;

    class GAZEBO_VISIBLE PresetManager
    {

      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: PresetManager(WorldPtr _world, sdf::ElementPtr _sdf);

      /// \brief Destructor.
      public: ~PresetManager();

      public: bool SetCurrentProfile(const std::string& _name);

      public: Preset* GetCurrentProfile() const;

      public: std::string GetCurrentProfileName() const;
 
      public: std::vector<std::string> GetAllProfileNames() const;

      public: std::vector<Preset*> GetAllProfiles() const;

      public: bool SetProfileParam(const std::string& _profileName,
                                   const std::string& _key,
                                   const boost::any &_value);

      public: bool SetCurrentProfileParam(const std::string& _key,
                                          const boost::any &_value);

      public: void CreateProfile(const std::string& _name);

      public: void CreateProfileFromPreset(const std::string& _name,
                                           Preset* _preset);

      public: sdf::ElementPtr GetSDFForProfile(const std::string &_name) const;

      public: void SetSDFForProfile(const std::string &_name,
                sdf::ElementPtr _sdf);

      private: void SetPresetFromSDF(sdf::ElementPtr _elem, Preset* _paramMap);

      private: PresetManagerPrivate *dataPtr;
    };
  }  // namespace physics
}  //namespace gazebo

#endif
