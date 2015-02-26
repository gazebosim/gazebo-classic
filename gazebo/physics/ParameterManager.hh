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

// TODO: PIMPL

#ifndef _PARAMETERMANAGER_H_
#define _PARAMETERMANAGER_H_

#include "gazebo/util/system.hh"
#include "gazebo/physics/PhysicsEngine.hh"

namespace gazebo
{
  namespace physics
  { 
    // Might want to make this a protobuf message.
    class PhysicsProfile
    {
      public: PhysicsProfile();

      public: PhysicsProfile(const std::string &_name,
                  const std::string &_physicsEngineName);

      public: void SetParam(const std::string &_key,
                  const boost::any &_value);

      public: void SetName(const std::string &_name);

      public: std::string GetName() const;

      public: void SetPhysicsEngine(const std::string &_name);

      public: std::string GetName() const;

      private: std::string name; // necessary?

      private: std::string physicsEngineName;

      private: std::map<std::string, boost::any> paramMap;
    }

    class GAZEBO_VISIBLE ParameterManager 
    {
      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      public: ParameterManager(WorldPtr _world);

      public: void SaveWorldToProfile(const std::string &_profileName);

      public: bool ApplyPhysicsProfile(const std::string &_profileName);

      public: bool ApplyPhysicsProfile(int i);

      public: PhysicsProfile GetProfile(const std::string &_profileName);

      public: std::vector<PhysicsProfile> GetProfiles();

      public: PhysicsProfile *GetCurrentProfile();

      private: PhysicsEngine *physicsEngine;

      private: WorldPtr world;

      private: std::map<std::string, PhysicsProfile> physicsProfiles;

      private: PhysicsProfile *currentProfile;
    }
  } // namespace gazebo
} // namespace physics

#endif
