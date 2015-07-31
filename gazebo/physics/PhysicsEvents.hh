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
#ifndef _GAZEBO_PHYSICS_EVENTS_HH_
#define _GAZEBO_PHYSICS_EVENTS_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GAZEBO_VISIBLE Events
    {
      /////////////////////////////////////////////////
      /// \brief Connect to the update physics end event.
      /// Be very cautious when using this event, as it is triggered every
      /// time physics is updated.
      public: template<typename T>
              static event::ConnectionPtr ConnectUpdatePhysicsEnd(T _subscriber)
              { return updatePhysicsEnd.Connect(_subscriber); }

      /// \brief Disconnect from the update physics end event
      public: static void DisconnectUpdatePhysicsEnd(
                  event::ConnectionPtr _subscriber)
              { updatePhysicsEnd.Disconnect(_subscriber); }

      /// \brief Physics update end event.
      public: static event::EventT<void ()> updatePhysicsEnd;
    };
  }
}
#endif
