/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#ifndef _EVENT_PRIVATE_HH_
#define _EVENT_PRIVATE_HH_

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <vector>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Time.hh"

namespace gazebo
{
  namespace event
  {
    class Event;
    class Connection;

    /// \internal
    // Private data members for Event class.
    class EventPrivate
    {
      // \brief Constructor
      public: EventPrivate();

      /// \brief True if the event has been signaled.
      public: bool signaled;
    };

    /// \internal
    // Private data members for Connection class.
    class ConnectionPrivate
    {
      /// \brief Constructor.
      public: ConnectionPrivate();

      /// \brief Constructor.
      /// \param[in] _e Event pointer to connect with
      /// \param[in] _i Unique id
      public: ConnectionPrivate(Event *_e, int _i);

      /// \brief the event for this connection
      public: Event *event;

      /// \brief the id set in the constructor
      public: int id;

      /// \brief set during the constructor
      public: common::Time creationTime;
    };

    /// \internal
    // Private data members for EventT<T> class.
    template< typename T>
    class EventTPrivate : public EventPrivate
    {
      /// \def EvtConnectionMap
      /// \brief Event Connection map typedef.
      typedef std::map<int, boost::function<T>*> EvtConnectionMap;

      /// \brief array of connection callbacks
      public: EvtConnectionMap connections;

      /// \brief Set of connections to erased.
      public: std::vector<int> connectionsToErase;

      /// \brief a thread lock
      public: boost::mutex connectionsEraseMutex;
    };
  }
}
#endif
