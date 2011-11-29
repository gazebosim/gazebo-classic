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
  namespace gui
  {
    class Events
    {
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEntity( T subscriber )
              { return createEntity.Connect(subscriber); }

      public: static void DisconnectCreateEntity( event::ConnectionPtr subscriber)
              { createEntity.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the move mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMoveMode( T subscriber )
              { return moveMode.Connect(subscriber); }

      public: static void DisconnectMoveMode( event::ConnectionPtr subscriber)
              { moveMode.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the manip mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectManipMode( T subscriber )
              { return manipMode.Connect(subscriber); }

      public: static void DisconnectManipMode( event::ConnectionPtr subscriber)
              { manipMode.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the fullscreen signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFullScreen(T subscriber)
              { return fullScreen.Connect(subscriber); }

      public: static void DisconnectFullScreen( event::ConnectionPtr subscriber)
              { fullScreen.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view FPS signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFPS(T subscriber)
              { return fps.Connect(subscriber); }

      public: static void DisconnectFPS( event::ConnectionPtr subscriber)
              { fps.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view Orbit signal
      public: template<typename T>
              static event::ConnectionPtr ConnectOrbit(T subscriber)
              { return orbit.Connect(subscriber); }

      public: static void DisconnectOrbit( event::ConnectionPtr subscriber)
              { orbit.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view KeyPress signal
      public: template<typename T>
              static event::ConnectionPtr ConnectKeyPress(T subscriber)
              { return keyPress.Connect(subscriber); }

      public: static void DisconnectKeyPress( event::ConnectionPtr subscriber)
              { keyPress.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      public: template<typename T>
              static event::ConnectionPtr ConnectModelUpdate(T subscriber)
              { return modelUpdate.Connect(subscriber); }

      public: static void DisconnectModelUpdate(event::ConnectionPtr subscriber)
              { modelUpdate.Disconnect(subscriber); }




      ///  that indicates the user is moving the camera
      public: static event::EventT<void (bool)>  moveMode;

      ///  that indicates the user is manipulating an object
      public: static event::EventT<void (bool)>  manipMode;

      public: static event::EventT<void (std::string)> createEntity;
      public: static event::EventT<void (const msgs::Model &)> modelUpdate;
      public: static event::EventT<void (bool)> fullScreen;
      public: static event::EventT<void ()> fps;
      public: static event::EventT<void ()> orbit;

      public: static event::EventT<void (std::string)> keyPress;
    };
  }
}
