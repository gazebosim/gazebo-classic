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
#ifndef _GAZEBO_PHYSICS_PRESETMANAGER_PRIVATE_HH_
#define _GAZEBO_PHYSICS_PRESETMANAGER_PRIVATE_HH_

#include <map>
#include <string>
#include <mutex>
#include "gazebo/physics/PhysicsEngine.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data pointer for Preset class
    class PresetPrivate
    {
      /// \brief Constructor
      /// \param[in] _name The name of the preset profile.
      public: PresetPrivate(const std::string &_name = "default name")
          : name(_name)
      {
      }

      /// \brief Name of this preset profile
      public: std::string name;

      /// \brief Map of key, parameter pairs
      public: std::map<std::string, boost::any> parameterMap;

      /// \brief SDF for the physics element represented by this object
      public: sdf::ElementPtr elementSDF;
    };

    class Preset;

    /// \internal
    /// \brief Private data pointer for PresetManager class
    class PresetManagerPrivate
    {
      /// \brief Name of the current preset
      public: std::string currentPreset;

      /// \brief Map of all known preset profile pairs keyed by name
      public: std::map<std::string, Preset> presetProfiles;

      /// \brief Physics engine instrumented by this PresetManager
      public: PhysicsEnginePtr physicsEngine;

      /// \brief Mutex to protect setting the current preset profile.
      public: std::mutex currentProfileMutex;
    };
  }
}

#endif
