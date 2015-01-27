/*
 * Copyright (C) 2013-2014 Open Source Robotics Foundation
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
      class GAZEBO_VISIBLE Events
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

        /// \brief Connect a Gazebo event to the save signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveModelEditor(T _subscriber)
          { return saveModelEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the save signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveAsModelEditor
              (T _subscriber)
          { return saveAsModelEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the save signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveModelEditor(
            event::ConnectionPtr _subscriber)
          { saveModelEditor.Disconnect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the save as signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveAsModelEditor(
            event::ConnectionPtr _subscriber)
          { saveAsModelEditor.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the new signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectNewModelEditor(T _subscriber)
          { return newModelEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the new signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectNewModelEditor(
              event::ConnectionPtr _subscriber)
          { newModelEditor.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the exit signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectExitModelEditor(T _subscriber)
          { return exitModelEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectExitModelEditor(
            event::ConnectionPtr _subscriber)
          { exitModelEditor.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the model changed signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelChanged(T _subscriber)
          { return modelChanged.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the model changed signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectModelChanged(
            event::ConnectionPtr _subscriber)
          { modelChanged.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the name changed signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectModelNameChanged
              (T _subscriber)
          { return modelNameChanged.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void ConnectModelNameChanged(
            event::ConnectionPtr _subscriber)
          { modelNameChanged.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the name changed signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveModel
              (T _subscriber)
          { return saveModel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void ConnectSaveModel(
            event::ConnectionPtr _subscriber)
          { saveModel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the name changed signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectNewModel
              (T _subscriber)
          { return newModel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void ConnectNewModel(
            event::ConnectionPtr _subscriber)
          { newModel.Disconnect(_subscriber); }

        /// \brief A model has been completed and uploaded onto the server.
        public: static event::EventT<void ()> finishModel;

        /// \brief Request to save the model.
        public: static event::EventT<bool ()> saveModelEditor;

        /// \brief Request to save the model as.
        public: static event::EventT<bool ()> saveAsModelEditor;

        /// \brief Request to start a new model.
        public: static event::EventT<void ()> newModelEditor;

        /// \brief Request to exit the editor.
        public: static event::EventT<void ()> exitModelEditor;

        /// \brief Model has been changed.
        public: static event::EventT<void ()> modelChanged;

        /// \brief Name was changed in the editor palette.
        public: static event::EventT<void (std::string)> modelNameChanged;

        /// \brief Notify that model has been saved.
        public: static event::EventT<void (std::string)> saveModel;

        /// \brief Notify that model has been newed.
        public: static event::EventT<void ()> newModel;
      };
    }
  }
}
#endif
