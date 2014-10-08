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
#ifndef _BUILDING_EDITOR_EVENTS_HH_
#define _BUILDING_EDITOR_EVENTS_HH_

#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    namespace editor
    {
      class GAZEBO_VISIBLE Events
      {
        /// \brief Connect a boost::slot to the create editor item signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectCreateBuildingEditorItem(T _subscriber)
          { return createBuildingEditorItem.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the create editor item signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectCreateBuildingEditorItem(
            event::ConnectionPtr _subscriber)
          { createBuildingEditorItem.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the save model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveBuildingModel(T _subscriber)
          { return saveBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the save model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveBuildingModel(
            event::ConnectionPtr _subscriber)
          { saveBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectFinishBuildingModel(T _subscriber)
          { return finishBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectFinishBuildingModel(
            event::ConnectionPtr _subscriber)
          { finishBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the discard model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectDiscardBuildingModel(T _subscriber)
          { return discardBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the discard model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectDiscardBuildingModel(
            event::ConnectionPtr _subscriber)
          { discardBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the change model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectChangeBuildingLevel(T _subscriber)
          { return changeBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the change level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectChangeBuildingLevel(
            event::ConnectionPtr _subscriber)
          { changeBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the add level signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectAddBuildingLevel(T _subscriber)
          { return addBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the add level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectAddBuildingLevel(
            event::ConnectionPtr _subscriber)
          { addBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the delete level signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectDeleteBuildingLevel(T _subscriber)
          { return deleteBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the delete level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectDeleteBuildingLevel(
              event::ConnectionPtr _subscriber)
          { deleteBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the change level name signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectChangeBuildingLevelName(T _subscriber)
          { return changeBuildingLevelName.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the change level name signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectChangeBuildingLevelName(
              event::ConnectionPtr _subscriber)
          { changeBuildingLevelName.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the change zoom signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
          static event::ConnectionPtr
              ConnectChangeBuildingEditorZoom(T _subscriber)
        { return changeBuildingEditorZoom.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the change zoom level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectChangeBuildingEditorZoom(
            event::ConnectionPtr _subscriber)
          { changeBuildingEditorZoom.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the save signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveBuildingEditor(T _subscriber)
          { return saveBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the save signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveBuildingEditor(
            event::ConnectionPtr _subscriber)
          { saveBuildingEditor.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the done signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectDoneBuildingEditor(T _subscriber)
          { return doneBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the done signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectDoneBuildingEditor(
                    event::ConnectionPtr _subscriber)
                { doneBuildingEditor.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the discard signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectDiscardBuildingEditor(T _subscriber)
          { return discardBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the discard signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectDiscardBuildingEditor(
              event::ConnectionPtr _subscriber)
          { discardBuildingEditor.Disconnect(_subscriber); }

        /// \brief Connect a boost::slot to the exit signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectExitBuildingEditor(T _subscriber)
          { return exitBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a boost::slot to the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectExitBuildingEditor(
            event::ConnectionPtr _subscriber)
          { exitBuildingEditor.Disconnect(_subscriber); }

        /// \brief An editor item is to be created
        public: static event::EventT<void (std::string)>
            createBuildingEditorItem;

        /// \brief A model has been saved with a name and a location
        public: static event::EventT<void (std::string, std::string)>
            saveBuildingModel;

        /// \brief A model has been completed and uploaded onto the server.
        public: static event::EventT<void ()> finishBuildingModel;

        /// \brief A model has been discarded
        public: static event::EventT<void ()> discardBuildingModel;

        /// \brief The current level has been changed
        public: static event::EventT<void (int)> changeBuildingLevel;

        /// \brief A new level has been added
        public: static event::EventT<void ()> addBuildingLevel;

        /// \brief A new level has been deleted
        public: static event::EventT<void (int)> deleteBuildingLevel;

        /// \brief The current level name has been changed
        public: static event::EventT<void (int, std::string)>
            changeBuildingLevelName;

        /// \brief The current zoom level has been changed
        public: static event::EventT<void (double)> changeBuildingEditorZoom;

        /// \brief Save the model
        public: static event::EventT<void ()> saveBuildingEditor;

        /// \brief Discard the model
        public: static event::EventT<void ()> discardBuildingEditor;

        /// \brief Finish creating the model, save, and exit
        public: static event::EventT<void ()> doneBuildingEditor;

        /// \brief Exit the editor mode with the option to save
        public: static event::EventT<void ()> exitBuildingEditor;
      };
    }
  }
}
#endif
