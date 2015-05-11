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

#ifndef _GAZEBO_EVENT_PLUGIN_HH_
#define _GAZEBO_EVENT_PLUGIN_HH_

#include <sdf/sdf.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE EventPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: EventPlugin();

    /// \brief Destructor.
    public: ~EventPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private: void Update();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief SDF pointer.
    private: sdf::ElementPtr sdf;

    private: class Region
             {
               public: std::string name;

               public: math::Pose pose;
               public: math::Vector3 box;

               public: std::string msg;

               public: transport::PublisherPtr pub;
             };


    private: std::map<std::string, Region*> regions;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: transport::NodePtr node;
  };
}
#endif
