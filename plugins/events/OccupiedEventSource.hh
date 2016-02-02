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

#ifndef _GAZEBO_PLUGINS_EVENTS_OCCUPIED_EVENT_SOURCE_HH_
#define _GAZEBO_PLUGINS_EVENTS_OCCUPIED_EVENT_SOURCE_HH_

#include <string>
#include <map>

#include <sdf/sdf.hh>

#include <gazebo/transport/TransportTypes.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

#include "Region.hh"
#include "EventSource.hh"

namespace gazebo
{
  /// \brief A plugin that transmits a message when an in-region event occurs.
  /// Events are specified in SDF. The following is example usage:
  //
  /// \verbatim
  ///    <plugin filename="libSimEventsPlugin.so" name="event_plugin">
  ///      <region>
  ///        <name>region1</name>
  ///        <volume>
  ///          <min>1.5 -1 0</min>
  ///          <max>2.5 1 1</max>
  ///        </volume>
  ///      </region>
  ///
  ///      <event>
  ///        <name>region1_event</name>
  ///        <type>occupied</type>
  ///        <topic>~/elevator</topic>
  ///        <region>region1</region>
  ///        <msg_data>0</msg_data>
  ///      </event>
  ///   </plugin>
  /// \endverbatim
  class GAZEBO_VISIBLE OccupiedEventSource : public EventSource
  {
    // Documentation inherited
    public: OccupiedEventSource(transport::PublisherPtr _pub,
                physics::WorldPtr _world,
                const std::map<std::string, RegionPtr> &_regions);

    /// \brief Destructor.
    public: ~OccupiedEventSource() = default;

    // Documentation inherited
    public: virtual void Load(const sdf::ElementPtr _sdf);

    /// \brief Update function called once every cycle
    private: void Update();

    /// \brief SDF pointer.
    private: sdf::ElementPtr sdf;

    /// \brief Map of region names to regions.
    private: std::map<std::string, RegionPtr> regions;

    /// \brief String message that is transmitted when an event occurs.
    public: msgs::GzString msg;

    /// \brief Publisher that transmits the message when an event occurs.
    public: transport::PublisherPtr msgPub;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to a transport node.
    private: transport::NodePtr node;

    /// \brief The region used for the in region check.
    private: std::string regionName;
  };
}
#endif
