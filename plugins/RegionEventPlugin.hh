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

#ifndef _GAZEBO_REGION_EVENT_PLUGIN_HH_
#define _GAZEBO_REGION_EVENT_PLUGIN_HH_

#include <string>
#include <map>

#include <sdf/sdf.hh>

#include <gazebo/transport/TransportTypes.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// \brief A plugin that transmits a message when an in-region event occurs.
  /// Events are specified in SDF. The following is example usage:
  //
  /// \verbatim
  ///    <plugin filename="libRegionEventPlugin.so" name="event_plugin">
  ///    <region name="region1">
  ///      <pose>2 0 0.5 0 0 0</pose>
  ///      <box>1 2 1</box>
  ///      <on_trigger>
  ///        <topic>~/my_topic_name</topic>
  ///        <msg type="string">
  ///          <data>0</data>
  ///        </msg>
  ///      </on_trigger>
  ///    </region>
  /// \endverbatim
  class GAZEBO_VISIBLE RegionEventPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: RegionEventPlugin() = default;

    /// \brief Destructor.
    public: ~RegionEventPlugin() = default;

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Update function called once every cycle
    private: void Update();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief SDF pointer.
    private: sdf::ElementPtr sdf;

    /// \brief Description of a region that will trigger an event.
    private: class Region
             {
               /// \brief Name of the region
               public: std::string name;

               /// \brief Center pose of the region
               public: math::Pose pose;

               /// \brief Box that encapsulates the region. The box is
               /// centered at the pose.
               public: math::Vector3 box;

               /// \brief String message that is transmitted when an event
               /// occurs.
               public: std::string msg;

               /// \brief Publisher that transmits the message when an event
               /// occurs.
               public: transport::PublisherPtr pub;
             };


    /// \brief Map of region names to regions.
    private: std::map<std::string, Region*> regions;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to a transport node.
    private: transport::NodePtr node;
  };
}
#endif
