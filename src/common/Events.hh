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

#include <string>
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
              static ConnectionPtr ConnectPause(T _subscriber)
              { return pause.Connect(_subscriber); }
      public: static void DisconnectPause(ConnectionPtr _subscriber)
              { pause.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the step signal
      public: template<typename T>
              static ConnectionPtr ConnectStep(T _subscriber)
              { return step.Connect(_subscriber); }
      public: static void DisconnectStep(ConnectionPtr _subscriber)
              { step.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the quit signal
      public: template<typename T>
              static ConnectionPtr ConnectQuit(T _subscriber)
              { return quit.Connect(_subscriber); }
      public: static void DisconnectQuit(ConnectionPtr _subscriber)
              { quit.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world created signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldCreated(T _subscriber)
              { return worldCreated.Connect(_subscriber); }
      public: static void DisconnectWorldCreated(ConnectionPtr _subscriber)
              { worldCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return entityCreated.Connect(_subscriber); }
      public: static void DisconnectCreateEntity(ConnectionPtr _subscriber)
              { entityCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the set selected entity
      public: template<typename T>
              static ConnectionPtr ConnectSetSelectedEntity(T _subscriber)
              { return setSelectedEntity.Connect(_subscriber); }
      public: static void DisconnectSetSelectedEntity(ConnectionPtr _subscriber)
              { setSelectedEntity.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the delete entity
      public: template<typename T>
              static ConnectionPtr ConnectDeleteEntity(T _subscriber)
              { return deleteEntity.Connect(_subscriber); }
      public: static void DisconnectDeleteEntity(ConnectionPtr _subscriber)
              { deleteEntity.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectAddEntity(T _subscriber)
              { return addEntity.Connect(_subscriber); }
      public: static void DisconnectAddEntity(ConnectionPtr _subscriber)
              { addEntity.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show light source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowLights(T _subscriber)
              { return showLights.Connect(_subscriber); }
      public: static void DisconnectShowLights(ConnectionPtr _subscriber)
              { showLights.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show camera source signal
      public: template<typename T>
              static ConnectionPtr ConnectShowCameras(T _subscriber)
              { return showCameras.Connect(_subscriber); }
      public: static void DisconnectShowCameras(ConnectionPtr _subscriber)
              { showCameras.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show contacts signal
      public: template<typename T>
              static ConnectionPtr ConnectShowContacts(T _subscriber)
              { return showContacts.Connect(_subscriber); }
      public: static void DisconnectShowContacts(ConnectionPtr _subscriber)
              { showContacts.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show wireframe signal
      public: template<typename T>
              static ConnectionPtr ConnectShowWireframe(T _subscriber)
              { return wireframe.Connect(_subscriber); }
      public: static void DisconnectShowWireframe(ConnectionPtr _subscriber)
              { wireframe.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show physics signal
      public: template<typename T>
              static ConnectionPtr ConnectShowPhysics(T _subscriber)
              { return showPhysics.Connect(_subscriber); }
      public: static void DisconnectShowPhysics(ConnectionPtr _subscriber)
              { showPhysics.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show joints signal
      public: template<typename T>
              static ConnectionPtr ConnectShowJoints(T _subscriber)
              { return showJoints.Connect(_subscriber); }
      public: static void DisconnectShowJoints(ConnectionPtr _subscriber)
              { showJoints.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the show bounding boxes signal
      public: template<typename T>
              static ConnectionPtr ConnectShowBoundingBoxes(T _subscriber)
              { return showBoundingBoxes.Connect(_subscriber); }
      public: static void DisconnectShowBoundingBoxes(ConnectionPtr _subscriber)
              { showBoundingBoxes.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update start signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateStart(T _subscriber)
              { return worldUpdateStart.Connect(_subscriber); }
      public: static void DisconnectWorldUpdateStart(ConnectionPtr _subscriber)
              { worldUpdateStart.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update end signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateEnd(T _subscriber)
              { return worldUpdateEnd.Connect(_subscriber); }
      public: static void DisconnectWorldUpdateEnd(ConnectionPtr _subscriber)
              { worldUpdateEnd.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the entity selected signal
      public: template<typename T>
              static ConnectionPtr ConnectEntitySelected(T _subscriber)
              { return entitySelected.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the entity selected signal
      public: static void DisconnectEntitySelected(ConnectionPtr _subscriber)
              { entitySelected.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Render start signal
      public: template<typename T>
              static ConnectionPtr ConnectPreRender(T _subscriber)
              { return preRender.Connect(_subscriber); }
      /// \brief Disconnect a render start signal
      public: static void DisconnectPreRender(ConnectionPtr _subscriber)
              { preRender.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the render update signal
      public: template<typename T>
              static ConnectionPtr ConnectRender(T _subscriber)
              { return render.Connect(_subscriber); }
      public: static void DisconnectRender(ConnectionPtr _subscriber)
              { render.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the post render update signal
      public: template<typename T>
              static ConnectionPtr ConnectPostRender(T _subscriber)
              { return postRender.Connect(_subscriber); }
      public: static void DisconnectPostRender(ConnectionPtr _subscriber)
              { postRender.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer start signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStart(T _subscriber)
              { return diagTimerStart.Connect(_subscriber); }
      public: static void DisconnectDiagTimerStart(ConnectionPtr _subscriber)
              { diagTimerStart.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer stop signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStop(T _subscriber)
              { return diagTimerStop.Connect(_subscriber); }
      public: static void DisconnectDiagTimerStop(ConnectionPtr _subscriber)
              { diagTimerStop.Disconnect(_subscriber); }

      /// \brief Pause signal
      public: static EventT<void (bool)> pause;

      /// \brief Step the simulation once signal
      public: static EventT<void ()> step;

      /// \brief Quit the simulation once signal
      public: static EventT<void ()> quit;

      /// \brief A world has been created
      public: static EventT<void (std::string)> worldCreated;

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

