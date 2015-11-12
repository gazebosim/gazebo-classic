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
#ifndef _GZ_TOOL_MARKER_HH_
#define _GZ_TOOL_MARKER_HH_

#include "gz.hh"

namespace gazebo
{
  /// \brief Marker command. This command line option to `gz` allows the
  /// creation and deletion of visual markers.
  class MarkerCommand : public Command
  {
    /// \brief Constructor
    public: MarkerCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    /// \brief List all the visual markers.
    private: void List() const;

    /// \brief Add or modify a marker.
    private: void Add(const std::string &_ns, const unsigned int _id,
    const std::string &_type, const common::Time _lifetime) const;

    /// \brief Node pointer.
    private: transport::NodePtr node;

    /// \brief Pointer to the marker publisher.
    private: transport::PublisherPtr pub;
  };
}
#endif
