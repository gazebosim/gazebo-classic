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

#ifndef _GAZEBO_TRANSPORTER_PLUGIN_HH_
#define _GAZEBO_TRANSPORTER_PLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE TransporterPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: TransporterPlugin();

    /// \brief Destructor.
    public: ~TransporterPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief World pointer.
    protected: physics::WorldPtr world;

    /// \brief SDF pointer.
    protected: sdf::ElementPtr sdf;

    private: class Pad
             {
               public: std::string name;
               public: std::string dest;

               public: math::Pose incomingPose;
               public: math::Pose outgoingPose;

               public: math::Vector3 incomingBox;
               public: math::Vector3 outgoingBox;
             };

    private: std::map<std::string, Pad*> pads;
  };
}
#endif
