/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "common/Event.hh"
#include "msgs/msgs.h"

namespace gazebo
{
  namespace rendering
  {
    class Events
    {
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateScene(T subscriber)
              { return createScene.Connect(subscriber); }

      public: static void DisconnectCreateScene(
                  event::ConnectionPtr subscriber)
              { createScene.Disconnect(subscriber); }

    public: template<typename T>
              static event::ConnectionPtr ConnectRemoveScene(T subscriber)
              { return removeScene.Connect(subscriber); }

      public: static void DisconnectRemoveScene(
                  event::ConnectionPtr subscriber)
              { removeScene.Disconnect(subscriber); }


      public: static event::EventT<void (const std::string &)> createScene;
      public: static event::EventT<void (const std::string &)> removeScene;
    };
  }
}

