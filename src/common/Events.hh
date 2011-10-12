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

#include "common/Event.hh"

namespace gazebo
{
  namespace event
  {
    class Events
    {
      /// \addtogroup gazebo_event
      /// \{
 
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the pause signal
      public: template<typename T>
              static ConnectionPtr ConnectPause( T subscriber )
              { return pause.Connect(subscriber); }

      public: static void DisconnectPause( ConnectionPtr subscriber)
              { pause.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the step signal
      public: template<typename T>
              static ConnectionPtr ConnectStep( T subscriber )
              { return step.Connect(subscriber); }

      public: static void DisconnectStep( ConnectionPtr subscriber)
              { step.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the quit signal
      public: template<typename T>
              static ConnectionPtr ConnectQuit( T subscriber )
              { return quit.Connect(subscriber); }

      public: static void DisconnectQuit( ConnectionPtr subscriber)
              { quit.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectCreateEntity( T subscriber )
              { return entityCreated.Connect(subscriber); }

      public: static void DisconnectCreateEntity( ConnectionPtr subscriber)
              { entityCreated.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the set selected entity
      public: template<typename T>
              static ConnectionPtr ConnectSetSelectedEntity( T subscriber )
              { return setSelectedEntity.Connect(subscriber); }

      public: static void DisconnectSetSelectedEntity(ConnectionPtr subscriber)
              { setSelectedEntity.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the delete entity
      public: template<typename T>
              static ConnectionPtr ConnectDeleteEntity( T subscriber )
              { return deleteEntity.Connect(subscriber); }

      public: static void DisconnectDeleteEntity( ConnectionPtr subscriber)
              { deleteEntity.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectAddEntity( T subscriber )
              { return addEntity.Connect(subscriber); }
      public: static void DisconnectAddEntity( ConnectionPtr subscriber)
              { addEntity.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show light source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowLights( T subscriber )
              { return showLights.Connect(subscriber); }

      public: static void DisconnectShowLights( ConnectionPtr subscriber )
              { showLights.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show camera source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowCameras( T subscriber )
              { return showCameras.Connect(subscriber); }
      public: static void DisconnectShowCameras( ConnectionPtr subscriber )
              { showCameras.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show contacts signal
      public: template<typename T>
              static ConnectionPtr ConnectShowContacts( T subscriber )
              { return showContacts.Connect(subscriber); }
      public: static void DisconnectShowContacts( ConnectionPtr subscriber )
              { showContacts.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show wireframe signal
      public: template<typename T>
              static ConnectionPtr ConnectShowWireframe( T subscriber )
              { return wireframe.Connect(subscriber); }
      public: static void DisconnectShowWireframe( ConnectionPtr subscriber )
              { wireframe.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show physics signal
      public: template<typename T>
              static ConnectionPtr ConnectShowPhysics( T subscriber )
              { return showPhysics.Connect(subscriber); }
      public: static void DisconnectShowPhysics( ConnectionPtr subscriber )
              { showPhysics.Disconnect(subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show joints signal
      public: template<typename T>
              static ConnectionPtr ConnectShowJoints( T subscriber )
              { return showJoints.Connect(subscriber); }
      public: static void DisconnectShowJoints( ConnectionPtr subscriber )
              { showJoints.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show bounding boxes signal
      public: template<typename T>
              static ConnectionPtr ConnectShowBoundingBoxes( T subscriber )
              { return showBoundingBoxes.Connect(subscriber); }
      public: static void DisconnectShowBoundingBoxes(ConnectionPtr subscriber)
              { showBoundingBoxes.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update start signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateStart(T subscriber)
              { return worldUpdateStart.Connect(subscriber); }
      public: static void DisconnectWorldUpdateStart(ConnectionPtr subscriber )
              { worldUpdateStart.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update end signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateEnd(T subscriber)
              { return worldUpdateEnd.Connect(subscriber); }
      public: static void DisconnectWorldUpdateEnd( ConnectionPtr subscriber )
              { worldUpdateEnd.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the entity selected signal
      public: template<typename T>
              static ConnectionPtr ConnectEntitySelected(T subscriber)
              { return entitySelected.Connect(subscriber); }
      /// \brief Disconnect a boost::slot the the entity selected signal
      public: static void DisconnectEntitySelected( ConnectionPtr subscriber )
              { entitySelected.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Render start signal
      public: template<typename T>
              static ConnectionPtr ConnectPreRender(T subscriber)
              { return preRender.Connect(subscriber); }

      /// \brief Disconnect a render start signal
      public: static void DisconnectPreRender( ConnectionPtr subscriber )
              { preRender.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the render update signal
      public: template<typename T>
              static ConnectionPtr ConnectRender( T subscriber )
              { return render.Connect(subscriber); }

      public: static void DisconnectRender( ConnectionPtr subscriber)
              { render.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the post render update signal
      public: template<typename T>
              static ConnectionPtr ConnectPostRender( T subscriber )
              { return postRender.Connect(subscriber); }

      public: static void DisconnectPostRender( ConnectionPtr subscriber)
              { postRender.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer start signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStart( T subscriber )
              { return diagTimerStart.Connect(subscriber); }

      public: static void DisconnectDiagTimerStart( ConnectionPtr subscriber)
              { diagTimerStart.Disconnect(subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer stop signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStop( T subscriber )
              { return diagTimerStop.Connect(subscriber); }

      public: static void DisconnectDiagTimerStop( ConnectionPtr subscriber)
              { diagTimerStop.Disconnect(subscriber); }


      /// \brief Pause signal
      public: static EventT<void (bool)> pause;

      /// \brief Step the simulation once signal
      public: static EventT<void ()> step;

      /// \brief Quit the simulation once signal
      public: static EventT<void ()> quit;

      /// \brief An entity has been created
      public: static EventT<void (std::string)> entityCreated;

      /// \brief An entity has been selected
      public: static EventT<void (std::string)> setSelectedEntity;

      /// \brief An entity has been added
      public: static EventT<void (std::string)> addEntity;

      /// \brief An entity has been deleted
      public: static EventT<void (std::string)> deleteEntity;

      /// \brief Light visuals should be shown
      public: static EventT<void (bool)> showLights;
      /// \brief Joint visuals should be shown
      public: static EventT<void (bool)> showJoints;
      /// \brief Camera visuals should be shown
      public: static EventT<void (bool)> showCameras;
      /// \brief Contact visuals should be shown
      public: static EventT<void (bool)> showContacts;
      /// \brief Wireframe enable signal
      public: static EventT<void (bool)> wireframe;
      /// \brief Show masses
      public: static EventT<void (bool)> showPhysics;
      /// \brief Show bounding boxes
      public: static EventT<void (bool)> showBoundingBoxes;

      /// \brief Entity has been selected
      public: static EventT<void (std::string)> entitySelected;

      /// \brief World update has started
      public: static EventT<void ()> worldUpdateStart;
      /// \brief World update has ended
      public: static EventT<void ()> worldUpdateEnd;

      /// \brief Pre-render
      public: static EventT<void ()> preRender;
      /// \brief Render
      public: static EventT<void ()> render;
      /// \brief Post-Render
      public: static EventT<void ()> postRender;

      /// \brief Diagnostic timer start
      public: static EventT<void (std::string)> diagTimerStart;
      /// \brief Diagnostic timer stop
      public: static EventT<void (std::string)> diagTimerStop;
    };
    /// \}
  }
}
#endif
