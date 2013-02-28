/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _GUI_EVENTS_HH_
#define _GUI_EVENTS_HH_

#include <string>
#include "common/Event.hh"
#include "msgs/msgs.hh"

namespace gazebo
{
  namespace gui
  {
    class Events
    {
      /////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return createEntity.Connect(_subscriber); }
      public: static void DisconnectCreateEntity(
                  event::ConnectionPtr _subscriber)
              { createEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the move mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMoveMode(T _subscriber)
              { return moveMode.Connect(_subscriber); }
      public: static void DisconnectMoveMode(event::ConnectionPtr _subscriber)
              { moveMode.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the manip mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectManipMode(T _subscriber)
              {return manipMode.Connect(_subscriber);}
      public: static void DisconnectManipMode(event::ConnectionPtr _subscriber)
              {manipMode.Disconnect(_subscriber);}

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the fullscreen signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFullScreen(T _subscriber)
              { return fullScreen.Connect(_subscriber); }
      public: static void DisconnectFullScreen(event::ConnectionPtr _subscriber)
              { fullScreen.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view FPS signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFPS(T _subscriber)
              { return fps.Connect(_subscriber); }
      public: static void DisconnectFPS(event::ConnectionPtr _subscriber)
              { fps.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view Orbit signal
      public: template<typename T>
              static event::ConnectionPtr ConnectOrbit(T _subscriber)
              { return orbit.Connect(_subscriber); }
      public: static void DisconnectOrbit(event::ConnectionPtr _subscriber)
              { orbit.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view KeyPress signal
      public: template<typename T>
              static event::ConnectionPtr ConnectKeyPress(T _subscriber)
              { return keyPress.Connect(_subscriber); }
      public: static void DisconnectKeyPress(event::ConnectionPtr _subscriber)
              { keyPress.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      public: template<typename T>
              static event::ConnectionPtr ConnectModelUpdate(T _subscriber)
              { return modelUpdate.Connect(_subscriber); }
      public: static void DisconnectModelUpdate(
                  event::ConnectionPtr _subscriber)
              { modelUpdate.Disconnect(_subscriber); }

      ///  that indicates the user is moving the camera
      public: static event::EventT<void (bool)>  moveMode;

      ///  that indicates the user is manipulating an object
      public: static event::EventT<void (std::string)>  manipMode;

      public: static event::EventT<void (std::string,
                                         std::string)> createEntity;

      public: static event::EventT<void (const msgs::Model &)> modelUpdate;
      public: static event::EventT<void (bool)> fullScreen;
      public: static event::EventT<void ()> fps;
      public: static event::EventT<void ()> orbit;

      public: static event::EventT<void (std::string)> keyPress;
    };
  }
}
#endif
