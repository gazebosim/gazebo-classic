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
#ifndef EVENTS_HH
#define EVENTS_HH

#include "Event.hh"

namespace gazebo
{
  class Entity;

  namespace event
  {
    class Events
    {

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the pause signal
      public: template<typename T>
              static ConnectionPtr ConnectPauseSignal( T subscriber )
              { return pauseSignal.Connect(subscriber); }

      public: static void DisconnectPauseSignal( ConnectionPtr subscriber)
              { pauseSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the step signal
      public: template<typename T>
              static ConnectionPtr ConnectStepSignal( T subscriber )
              { return stepSignal.Connect(subscriber); }

      public: static void DisconnectStepSignal( ConnectionPtr subscriber)
              { stepSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the quit signal
      public: template<typename T>
              static ConnectionPtr ConnectQuitSignal( T subscriber )
              { return quitSignal.Connect(subscriber); }

      public: static void DisconnectQuitSignal( ConnectionPtr subscriber)
              { quitSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectCreateEntitySignal( T subscriber )
              { return createEntitySignal.Connect(subscriber); }

      public: static void DisconnectCreateEntitySignal( ConnectionPtr subscriber)
              { createEntitySignal.Disconnect(subscriber); }


      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the move mode signal
      public: template<typename T>
              static ConnectionPtr ConnectMoveModeSignal( T subscriber )
              { return moveModeSignal.Connect(subscriber); }

      public: static void DisconnectMoveModeSignal( ConnectionPtr subscriber)
              { moveModeSignal.Disconnect(subscriber); }


      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the manip mode signal
      public: template<typename T>
              static ConnectionPtr ConnectManipModeSignal( T subscriber )
              { return manipModeSignal.Connect(subscriber); }

      public: static void DisconnectManipModeSignal( ConnectionPtr subscriber)
              { manipModeSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the set selected entity
      public: template<typename T>
              static ConnectionPtr ConnectSetSelectedEntitySignal( T subscriber )
              { return setSelectedEntitySignal.Connect(subscriber); }

      public: static void DisconnectSetSelectedEntitySignal(ConnectionPtr subscriber)
              { setSelectedEntitySignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the delete entity
      public: template<typename T>
              static ConnectionPtr ConnectDeleteEntitySignal( T subscriber )
              { return deleteEntitySignal.Connect(subscriber); }

      public: static void DisconnectDeleteEntitySignal( ConnectionPtr subscriber)
              { deleteEntitySignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectAddEntitySignal( T subscriber )
              { return addEntitySignal.Connect(subscriber); }
      public: static void DisconnectAddEntitySignal( ConnectionPtr subscriber)
              { addEntitySignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show light source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowLightsSignal( T subscriber )
              { return showLightsSignal.Connect(subscriber); }

      public: static void DisconnectShowLightsSignal( ConnectionPtr subscriber )
              { showLightsSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show camera source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowCamerasSignal( T subscriber )
              { return showCamerasSignal.Connect(subscriber); }
      public: static void DisconnectShowCamerasSignal( ConnectionPtr subscriber )
              { showCamerasSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show contacts signal
      public: template<typename T>
              static ConnectionPtr ConnectShowContactsSignal( T subscriber )
              { return showContactsSignal.Connect(subscriber); }
      public: static void DisconnectShowContactsSignal( ConnectionPtr subscriber )
              { showContactsSignal.Disconnect(subscriber); }


      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show wireframe signal
      public: template<typename T>
              static ConnectionPtr ConnectShowWireframeSignal( T subscriber )
              { return wireframeSignal.Connect(subscriber); }
      public: static void DisconnectShowWireframeSignal( ConnectionPtr subscriber )
              { wireframeSignal.Disconnect(subscriber); }


      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show physics signal
      public: template<typename T>
              static ConnectionPtr ConnectShowPhysicsSignal( T subscriber )
              { return showPhysicsSignal.Connect(subscriber); }
      public: static void DisconnectShowPhysicsSignal( ConnectionPtr subscriber )
              { showPhysicsSignal.Disconnect(subscriber); }


      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show joints signal
      public: template<typename T>
              static ConnectionPtr ConnectShowJointsSignal( T subscriber )
              { return showJointsSignal.Connect(subscriber); }
      public: static void DisconnectShowJointsSignal( ConnectionPtr subscriber )
              { showJointsSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show bounding boxes signal
      public: template<typename T>
              static ConnectionPtr ConnectShowBoundingBoxesSignal( T subscriber )
              { return showBoundingBoxesSignal.Connect(subscriber); }
      public: static void DisconnectShowBoundingBoxesSignal(ConnectionPtr subscriber)
              { showBoundingBoxesSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update start signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateStartSignal(T subscriber)
              { return worldUpdateStartSignal.Connect(subscriber); }
      public: static void DisconnectWorldUpdateStartSignal(ConnectionPtr subscriber )
              { worldUpdateStartSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update end signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateEndSignal(T subscriber)
              { return worldUpdateEndSignal.Connect(subscriber); }
      public: static void DisconnectWorldUpdateEndSignal( ConnectionPtr subscriber )
              { worldUpdateEndSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the entity selected signal
      public: template<typename T>
              static ConnectionPtr ConnectEntitySelectedSignal(T subscriber)
              { return entitySelectedSignal.Connect(subscriber); }
      /// \brief Disconnect a boost::slot the the entity selected signal
      public: static void DisconnectEntitySelectedSignal( ConnectionPtr subscriber )
              { entitySelectedSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Render start signal
      public: template<typename T>
              static ConnectionPtr ConnectPreRenderSignal(T subscriber)
              { return preRenderSignal.Connect(subscriber); }

      /// \brief Disconnect a render start signal
      public: static void DisconnectPreRenderSignal( ConnectionPtr subscriber )
              { preRenderSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the render update signal
      public: template<typename T>
              static ConnectionPtr ConnectRenderSignal( T subscriber )
              { return renderSignal.Connect(subscriber); }

      public: static void DisconnectRenderSignal( ConnectionPtr subscriber)
              { renderSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the post render update signal
      public: template<typename T>
              static ConnectionPtr ConnectPostRenderSignal( T subscriber )
              { return postRenderSignal.Connect(subscriber); }

      public: static void DisconnectPostRenderSignal( ConnectionPtr subscriber)
              { postRenderSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer start signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStartSignal( T subscriber )
              { return diagTimerStartSignal.Connect(subscriber); }

      public: static void DisconnectDiagTimerStartSignal( ConnectionPtr subscriber)
              { diagTimerStartSignal.Disconnect(subscriber); }

      ////////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer stop signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStopSignal( T subscriber )
              { return diagTimerStopSignal.Connect(subscriber); }

      public: static void DisconnectDiagTimerStopSignal( ConnectionPtr subscriber)
              { diagTimerStopSignal.Disconnect(subscriber); }


      public: static EventT<void (bool)> pauseSignal;
      public: static EventT<void ()> stepSignal;
      public: static EventT<void ()> quitSignal;

      public: static EventT<void (bool)>  moveModeSignal;
      public: static EventT<void (bool)>  manipModeSignal;

      public: static EventT<void (std::string)> createEntitySignal;
      public: static EventT<void (std::string)> setSelectedEntitySignal;
      public: static EventT<void (std::string)> addEntitySignal;
      public: static EventT<void (std::string)> deleteEntitySignal;

      public: static EventT<void (bool)> showLightsSignal;
      public: static EventT<void (bool)> showJointsSignal;
      public: static EventT<void (bool)> showCamerasSignal;
      public: static EventT<void (bool)> showContactsSignal;
      public: static EventT<void (bool)> wireframeSignal;
      public: static EventT<void (bool)> showPhysicsSignal;
      public: static EventT<void (bool)> showBoundingBoxesSignal;

      public: static EventT<void (Entity*)> entitySelectedSignal;

      public: static EventT<void ()> worldUpdateStartSignal;
      public: static EventT<void ()> worldUpdateEndSignal;

      public: static EventT<void ()> preRenderSignal;
      public: static EventT<void ()> renderSignal;
      public: static EventT<void ()> postRenderSignal;

      public: static EventT<void (std::string)> diagTimerStartSignal;
      public: static EventT<void (std::string)> diagTimerStopSignal;
    };
  }

}


#endif
