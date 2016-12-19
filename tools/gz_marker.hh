/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_TOOLS_MARKER_HH_
#define GAZEBO_TOOLS_MARKER_HH_

#include <string>
#include <ignition/transport/Node.hh>
#include "gz.hh"

namespace gazebo
{
  /// \brief Marker command. This command line option to `gz` allows the
  /// creation, deletion, and modificiation of visual markers.
  class MarkerCommand : public Command
  {
    /// \brief Constructor
    public: MarkerCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    /// \brief List all the visual markers.
    private: void List();

    /// \brief Add or modify a marker.
    /// \param[in] _ns Namespace for the marker
    /// \param[in] _id Marker id
    /// \param[in] _type Shape type. See the Type enum in
    /// ignition::msgs::Marker protobuf message
    /// \param[in] _lifetime Length of time the marker should be visible.
    /// \param[in] _parent Name of a parent visual to attach a marker to.
    /// \param[in] _layer Layer to add this marker to.
    private: void Add(const std::string &_ns, const unsigned int _id,
                 const std::string &_type, const common::Time &_lifetime,
                 const std::string &_parent, const int32_t _layer);

    /// \brief Send a marker message
    /// \param[in] _msg String representation of a marker protobuf message.
    private: void Msg(const std::string &_msg);

    /// \brief Delete a marker.
    /// \param[in] _ns Namespace for the marker
    /// \param[in] _id Marker id
    private: void Delete(const std::string &_ns, const unsigned int _id);

    /// \brief Delete all markers
    /// \param[in] _ns The namespace in which all markers should be deleted.
    /// If empty, all markers in all namespaces will be deleted.
    private: void DeleteAll(const std::string &_ns);

    /// \brief Ignition node.
    private: ignition::transport::Node node;
  };
}
#endif
