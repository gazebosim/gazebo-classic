/*
 * Copyright 2013 Open Source Robotics Foundation
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
#ifndef _MODEL_EDITOR_EVENTS_HH_
#define _MODEL_EDITOR_EVENTS_HH_

#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    namespace model
    {
      class Events
      {
        /// \brief Connect a boost::slot to the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectFinishModel(T _subscriber)
          { return finishModel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectFinishModel(
            event::ConnectionPtr _subscriber)
          { finishModel.Disconnect(_subscriber); }

        /// \brief A model has been completed and uploaded onto the server.
        public: static event::EventT<void ()> finishModel;
      };
    }
  }
}
#endif
