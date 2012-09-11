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
      /// \brief Disconnect a boost::slot the the pause signal
      public: static void DisconnectPause(ConnectionPtr _subscriber)
              { pause.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the step signal
      public: template<typename T>
              static ConnectionPtr ConnectStep(T _subscriber)
              { return step.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the step signal
      public: static void DisconnectStep(ConnectionPtr _subscriber)
              { step.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the stop signal
      public: template<typename T>
              static ConnectionPtr ConnectStop(T _subscriber)
              { return stop.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the stop signal
      public: static void DisconnectStop(ConnectionPtr _subscriber)
              { stop.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world created signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldCreated(T _subscriber)
              { return worldCreated.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the world created signal
      public: static void DisconnectWorldCreated(ConnectionPtr _subscriber)
              { worldCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return entityCreated.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the add entity signal
      public: static void DisconnectCreateEntity(ConnectionPtr _subscriber)
              { entityCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the set selected entity
      public: template<typename T>
              static ConnectionPtr ConnectSetSelectedEntity(T _subscriber)
              { return setSelectedEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the set selected entity
      public: static void DisconnectSetSelectedEntity(ConnectionPtr _subscriber)
              { setSelectedEntity.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the delete entity
      public: template<typename T>
              static ConnectionPtr ConnectDeleteEntity(T _subscriber)
              { return deleteEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the delete entity
      public: static void DisconnectDeleteEntity(ConnectionPtr _subscriber)
              { deleteEntity.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      public: template<typename T>
              static ConnectionPtr ConnectAddEntity(T _subscriber)
              { return addEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the add entity signal
      public: static void DisconnectAddEntity(ConnectionPtr _subscriber)
              { addEntity.Disconnect(_subscriber); }


      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update start signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateStart(T _subscriber)
              { return worldUpdateStart.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the world update start signal
      public: static void DisconnectWorldUpdateStart(ConnectionPtr _subscriber)
              { worldUpdateStart.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update end signal
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateEnd(T _subscriber)
              { return worldUpdateEnd.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the world update end signal
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
      /// \brief Disconnect a boost::slot the render update signal
      public: static void DisconnectRender(ConnectionPtr _subscriber)
              { render.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the post render update signal
      public: template<typename T>
              static ConnectionPtr ConnectPostRender(T _subscriber)
              { return postRender.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the post render update signal
      public: static void DisconnectPostRender(ConnectionPtr _subscriber)
              { postRender.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer start signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStart(T _subscriber)
              { return diagTimerStart.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the diagnostic timer start signal
      public: static void DisconnectDiagTimerStart(ConnectionPtr _subscriber)
              { diagTimerStart.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer stop signal
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStop(T _subscriber)
              { return diagTimerStop.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the diagnostic timer stop signal
      public: static void DisconnectDiagTimerStop(ConnectionPtr _subscriber)
              { diagTimerStop.Disconnect(_subscriber); }

      /// \brief Pause signal
      public: static EventT<void (bool)> pause;

      /// \brief Step the simulation once signal
      public: static EventT<void ()> step;

      /// \brief Simulation stop signal
      public: static EventT<void ()> stop;

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
