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

namespace gazebo
{
  namespace gui
  {
    class Events
    {
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEntitySignal( T subscriber )
              { return createEntitySignal.Connect(subscriber); }

      public: static void DisconnectCreateEntitySignal( event::ConnectionPtr subscriber)
              { createEntitySignal.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the move mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMoveModeSignal( T subscriber )
              { return moveModeSignal.Connect(subscriber); }

      public: static void DisconnectMoveModeSignal( event::ConnectionPtr subscriber)
              { moveModeSignal.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the manip mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectManipModeSignal( T subscriber )
              { return manipModeSignal.Connect(subscriber); }

      public: static void DisconnectManipModeSignal( event::ConnectionPtr subscriber)
              { manipModeSignal.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the fullscreen signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFullScreenSignal(T subscriber)
              { return fullScreenSignal.Connect(subscriber); }

      public: static void DisconnectFullScreenSignal( event::ConnectionPtr subscriber)
              { fullScreenSignal.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view FPS signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFPSSignal(T subscriber)
              { return fpsSignal.Connect(subscriber); }

      public: static void DisconnectFPSSignal( event::ConnectionPtr subscriber)
              { fpsSignal.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the view Orbit signal
      public: template<typename T>
              static event::ConnectionPtr ConnectOrbitSignal(T subscriber)
              { return orbitSignal.Connect(subscriber); }

      public: static void DisconnectOrbitSignal( event::ConnectionPtr subscriber)
              { orbitSignal.Disconnect(subscriber); }



      /// Signal that indicates the user is moving the camera
      public: static event::EventT<void (bool)>  moveModeSignal;

      /// Signal that indicates the user is manipulating an object
      public: static event::EventT<void (bool)>  manipModeSignal;

      public: static event::EventT<void (std::string)> createEntitySignal;
      public: static event::EventT<void (bool)> fullScreenSignal;
      public: static event::EventT<void ()> fpsSignal;
      public: static event::EventT<void ()> orbitSignal;
    };
  }
}
