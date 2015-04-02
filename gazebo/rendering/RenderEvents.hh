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
#ifndef _RENDER_EVENTS_HH_
#define _RENDER_EVENTS_HH_

#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Events Events.hh rendering/rendering.hh
    /// \brief Base class for rendering events
    class GAZEBO_VISIBLE Events
    {
      /// \brief Connect to a scene created event.
      /// \param[in] _subscriber Callback to trigger when event occurs.
      /// \return Pointer the connection. This must stay in scope.
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateScene(T _subscriber)
              { return createScene.Connect(_subscriber); }

      /// \brief Disconnect from a scene created event.
      /// \param[in] _connection The connection to disconnect.
      public: static void DisconnectCreateScene(
                  event::ConnectionPtr _connection)
              { createScene.Disconnect(_connection); }

      /// \brief Connect to a scene removed event.
      /// \param[in] _subscriber Callback to trigger when event occurs.
      /// \return Pointer the connection. This must stay in scope.
      public: template<typename T>
              static event::ConnectionPtr ConnectRemoveScene(T _subscriber)
              {return removeScene.Connect(_subscriber);}

      /// \brief Disconnect from a scene removed event.
      /// \param[in] _connection The connection to disconnect.
      public: static void DisconnectRemoveScene(
                  event::ConnectionPtr _connection)
              {removeScene.Disconnect(_connection);}

      /// \brief The event used to trigger a create scene event.
      public: static event::EventT<void (const std::string &)> createScene;

      /// \brief The event used to trigger a remve scene event.
      public: static event::EventT<void (const std::string &)> removeScene;
    };
    /// \}
  }
}
#endif
