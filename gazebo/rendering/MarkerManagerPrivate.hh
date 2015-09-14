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

#ifndef _GAZEBO_MARKER_MANAGER_PRIVATE_HH_
#define _GAZEBO_MARKER_MANAGER_PRIVATE_HH_

#include <string>
#include <map>
#include <list>
#include <mutex>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    class MarkerVisual;

    /// \def Marker_M
    /// \brief Map of markers. The first key is a marker namespace, the
    /// first value is the map of markers and their ids.
    typedef std::map<std::string, std::map<uint64_t, MarkerVisualPtr>> Marker_M;

    /// \def MarkerMsgs_L
    /// \brief List of marker messages.
    typedef std::list<boost::shared_ptr<msgs::Marker const> > MarkerMsgs_L;

    /// \internal
    /// Private data for the MarkerManager class
    class MarkerManagerPrivate
    {
      /// \brief Communication Node
      public: transport::NodePtr node;

      /// \brief Subscribe to marker topic
      public: transport::SubscriberPtr markerSub;

      /// \brief Map of markers
      public: Marker_M markers;

      /// \brief List of marker message to process.
      public: MarkerMsgs_L markerMsgs;

      /// \brief Connect to the prerender signal
      public: event::ConnectionPtr preRenderConnection;

      /// \brief Mutex to protect message list.
      public: std::mutex mutex;

      /// \brief Pointer to the scene
      public: ScenePtr scene;
    };
  }
}
#endif
