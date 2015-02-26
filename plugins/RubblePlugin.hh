/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RUBBLE_PLUGIN_HH_
#define _GAZEBO_RUBBLE_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE RubblePlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: RubblePlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    private: void MakeBox(const std::string &_name, math::Pose &_pose,
                          math::Vector3 &_size, double _mass);

    private: void MakeCinderBlock(const std::string &_name, math::Pose &_pose,
                                  math::Vector3 &_size, double _mass);

    // private: void MakeCylinder(const std::string &_name, math::Vector3 &_pos,
    //                           math::Vector3 &_size, double _mass);

    private: class Obj
             {
               public: math::Pose pose;
               public: math::Vector3 size;
               public: int type;
             };

    private: class CompoundObj
             {
               // center position
               public: math::Vector3 pos;

               // Total size
               public: math::Vector3 size;
               public: std::vector<Obj> objects;
             };

    // private: void MakeCompound(const std::string &_name, CompoundObj &_obj);
    private: physics::WorldPtr world;
  };
}
#endif
