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

#ifndef _GAZEBO_PHYSICS_PRESETMANAGER_HH_
#define _GAZEBO_PHYSICS_PRESETMANAGER_HH_

#include <string>
#include <map>
#include <vector>

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    class PresetPrivate;

    /// \class Preset PresetManager.hh gazebo/physics/PresetManager.hh
    /// \brief Representation of a preset physics profile

    class Preset
    {
      /// \brief Constructor.
      public: Preset();

      /// \brief Constructor
      /// \param[in] _name The name of the preset profile.
      public: Preset(const std::string & _name);

      /// \brief Get the profile name.
      /// \return The name of the preset profile.
      public: std::string Name() const;

      /// \brief Set the profile name.
      /// \param[in] _name The new of the preset profile.
      public: void Name(const std::string &_name);

      /// \brief Get a profile parameter.
      /// \param[in] _key The key of the parameter to retrieve.
      /// \param[out] The parameter value at the input key.
      /// \return True if the parameter exists in the map, false otherwise.
      public: bool GetParam(const std::string &_key, boost::any &_value) const;

      /// \brief Set a profile parameter.
      /// \param[in] _key The key of the parameter to change.
      /// \param[out] _value The new value of the parameter.
      public: void SetParam(const std::string &_key, const boost::any &_value);

      /// \brief Check if profile parameter is set.
      public: bool HasParam(const std::string &_key) const;

      /// \brief Set all parameters of this preset in the physics engine.
      /// \param[in] _physicsEngine The physics engine in which to affect the
      /// change.
      /// \return True if setting all parameters was successful.
      public: bool SetAllPhysicsParameters(PhysicsEnginePtr _physicsEngine)
          const;

      /// \brief Get this preset profile's SDF
      /// \return An SDF element pointer representing a <physics> element
      public: sdf::ElementPtr SDF() const;

      /// \brief Set this preset profile's SDF
      /// \param[in] _sdfElement Pointer to an SDF physics element.
      public: void SDF(sdf::ElementPtr _sdfElement);

      /// \brief Private data pointer for PIMPL
      private: PresetPrivate *dataPtr;
    };

    class PresetManagerPrivate;

    /// \class PresetManager PresetManager.hh gazebo/physics/PresetManager.hh
    /// \brief Class to manage preset physics profiles.
    class GAZEBO_VISIBLE PresetManager
    {
      /// \brief Constructor
      /// \param[in] _physicsEngine Pointer to the world physics engine.
      /// \param[in] _sdf Pointer to the world SDF element.
      public: PresetManager(PhysicsEnginePtr _physicsEngine,
          sdf::ElementPtr _sdf);

      /// \brief Destructor
      public: ~PresetManager();

      /// \brief Set the current profile.
      /// \param[in] _name The name of the new current profile.
      /// \return True if the profile switch was successful.
      public: bool CurrentProfile(const std::string &_name);

      /// \brief Get the name of the current profile.
      /// \return The name of the current profile.
      public: std::string CurrentProfile() const;

      /// \brief Get the name of all profiles.
      /// \return A vector containing all profile names.
      public: std::vector<std::string> AllProfiles() const;

      /// \brief Set a parameter for a certain profile.
      /// \param[in] _profileName The name of the profile to change.
      /// \param[in] _key The key of the parameter to change.
      /// \param[in] _value The value of the parameter to change.
      /// \return True if setting the parameter was successful.
      public: bool SetProfileParam(const std::string &_profileName,
                                   const std::string &_key,
                                   const boost::any &_value);

      /// \brief Get a parameter for a certain profile.
      /// \param[in] _name The name of the accessed profile.
      /// \param[in] _key The key of the accessed parameter.
      /// \param[out] _value The value of the accessed parameter.
      /// \return True if the parameter existed in profile "_name".
      public: bool GetProfileParam(const std::string &_name,
          const std::string &_key, boost::any &_value) const;

      /// \brief Set a parameter for the current profile.
      /// \param[in] _key The key of the parameter to be set.
      /// \param[in] _value The value of the parameter to be set.
      /// \return True if setting the parameter was successful.
      public: bool SetCurrentProfileParam(const std::string &_key,
                                       const boost::any &_value);

      /// \brief Get a parameter for the current profile.
      /// \param[in] _key The key of the accessed parameter.
      /// \param[out] _value The value of the accessed parameter.
      /// \return True if the parameter existed in profile "_name".
      public: bool GetCurrentProfileParam(const std::string &_key,
          boost::any &_value);

      /// \brief Create a new profile.
      /// \param[in] _name The name of the new profile.
      public: void CreateProfile(const std::string &_name);

      /// \brief Create a new profile from SDF. SDF determines the profile name
      /// \param[in] _sdf Pointer to a physics SDF element.
      /// \return The name of the new profile, read from SDF.
      public: std::string CreateProfile(sdf::ElementPtr _sdf);

      /// \brief Remove a profile.
      /// \param[in] _name The name of the profile to remove.
      public: void RemoveProfile(const std::string &_name);

      /// \brief Get the SDF for a profile.
      /// \param[in] _name The name of the profile to be accessed.
      /// \return Pointer to the SDF physics element representing the profile
      public: sdf::ElementPtr ProfileSDF(const std::string &_name) const;

      /// \brief Set the SDF for a profile.
      /// \param[in] _name The name of the profile to set.
      /// \param[in] _sdf The new SDF physics element for the profile.
      public: void ProfileSDF(const std::string &_name, sdf::ElementPtr _sdf);

      /// \brief Generate a Preset object from an SDF pointer
      /// \param[in] _sdf The SDF physics element for the profile.
      private: void GeneratePresetFromSDF(Preset *_preset,
          const sdf::ElementPtr _elem) const;

      /// \brief Get a pointer to the current profile preset.
      /// \return Pointer to the current profile preset object.
      private: Preset *CurrentPreset() const;

      /// \brief Private data pointer for PIMPL.
      private: PresetManagerPrivate *dataPtr;
    };
  }
}

#endif
