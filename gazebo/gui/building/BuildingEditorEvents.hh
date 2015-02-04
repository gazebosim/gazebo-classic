/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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
#include "gazebo/gui/qt.h"
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
        /// \brief Connect a Gazebo event to the toggle edit mode signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectToggleEditMode(T _subscriber)
          { return toggleEditMode.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the toggle edit mode signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectToggleEditMode(
            event::ConnectionPtr _subscriber)
          { toggleEditMode.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the create editor item signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectCreateBuildingEditorItem(T _subscriber)
          { return createBuildingEditorItem.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the create editor item signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectCreateBuildingEditorItem(
            event::ConnectionPtr _subscriber)
          { createBuildingEditorItem.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the color selected signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectColorSelected(T _subscriber)
          { return colorSelected.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the color selected signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectColorSelected(
            event::ConnectionPtr _subscriber)
          { colorSelected.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the texture selected signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectTextureSelected(T _subscriber)
          { return textureSelected.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the texture selected signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectTextureSelected(
            event::ConnectionPtr _subscriber)
          { textureSelected.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the save model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveBuildingModel(T _subscriber)
          { return saveBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the save model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveBuildingModel(
            event::ConnectionPtr _subscriber)
          { saveBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectFinishBuildingModel(T _subscriber)
          { return finishBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the finish model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectFinishBuildingModel(
            event::ConnectionPtr _subscriber)
          { finishBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the new model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectNewBuildingModel(T _subscriber)
          { return newBuildingModel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the new model signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectNewBuildingModel(
            event::ConnectionPtr _subscriber)
          { newBuildingModel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the change model signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectChangeBuildingLevel(T _subscriber)
          { return changeBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the change level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectChangeBuildingLevel(
            event::ConnectionPtr _subscriber)
          { changeBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the add level signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectAddBuildingLevel(T _subscriber)
          { return addBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the add level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectAddBuildingLevel(
            event::ConnectionPtr _subscriber)
          { addBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the delete level signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectDeleteBuildingLevel(T _subscriber)
          { return deleteBuildingLevel.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the delete level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectDeleteBuildingLevel(
              event::ConnectionPtr _subscriber)
          { deleteBuildingLevel.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the show floorplan signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectShowFloorplan(T _subscriber)
          { return showFloorplan.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the show floorplan signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectShowFloorplan(
              event::ConnectionPtr _subscriber)
          { showFloorplan.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the trigger show floorplan
        /// signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectTriggerShowFloorplan(T _subscriber)
          { return triggerShowFloorplan.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the trigger show floorplan
        /// signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectTriggerShowFloorplan(
              event::ConnectionPtr _subscriber)
          { triggerShowFloorplan.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the show elements signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectShowElements(T _subscriber)
          { return showElements.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the show elements signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectShowElements(
              event::ConnectionPtr _subscriber)
          { showElements.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the trigger show elements
        /// signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectTriggerShowElements(T _subscriber)
          { return triggerShowElements.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the trigger show elements
        /// signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectTriggerShowElements(
              event::ConnectionPtr _subscriber)
          { triggerShowElements.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the update level widget signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
                ConnectUpdateLevelWidget(T _subscriber)
          { return updateLevelWidget.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the update level widget signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectUpdateLevelWidget(
              event::ConnectionPtr _subscriber)
          { updateLevelWidget.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the change zoom signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
          static event::ConnectionPtr
              ConnectChangeBuildingEditorZoom(T _subscriber)
        { return changeBuildingEditorZoom.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the change zoom level signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectChangeBuildingEditorZoom(
            event::ConnectionPtr _subscriber)
          { changeBuildingEditorZoom.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the save signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveBuildingEditor(T _subscriber)
          { return saveBuildingEditor.Connect(_subscriber); }

        /// \brief Connect a Gazebo event to the save signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectSaveAsBuildingEditor
              (T _subscriber)
          { return saveAsBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the save signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveBuildingEditor(
            event::ConnectionPtr _subscriber)
          { saveBuildingEditor.Disconnect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the save as signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectSaveAsBuildingEditor(
            event::ConnectionPtr _subscriber)
          { saveAsBuildingEditor.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the new signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr
            ConnectNewBuildingEditor(T _subscriber)
          { return newBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the new signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectNewBuildingEditor(
              event::ConnectionPtr _subscriber)
          { newBuildingEditor.Disconnect(_subscriber); }

        /// \brief Connect a Gazebo event to the exit signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectExitBuildingEditor(T _subscriber)
          { return exitBuildingEditor.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void DisconnectExitBuildingEditor(
            event::ConnectionPtr _subscriber)
          { exitBuildingEditor.Disconnect(_subscriber); }

        /// \brief Toggle if the edit mode was checked or not.
        public: static event::EventT<void (bool)> toggleEditMode;

        /// \brief Connect a Gazebo event to the name changed signal
        /// \param[in] _subscriber the subscriber to this event
        /// \return a connection
        public: template<typename T>
            static event::ConnectionPtr ConnectBuildingNameChanged
              (T _subscriber)
          { return buildingNameChanged.Connect(_subscriber); }

        /// \brief Disconnect a Gazebo event from the exit signal
        /// \param[in] _subscriber the subscriber to this event
        public: static void ConnectBuildingNameChanged(
            event::ConnectionPtr _subscriber)
          { buildingNameChanged.Disconnect(_subscriber); }

        /// \brief An editor item is to be created
        public: static event::EventT<void (std::string)>
            createBuildingEditorItem;

        /// \brief A color has been selected.
        public: static event::EventT<void (QColor)> colorSelected;

        /// \brief A texture has been selected.
        public: static event::EventT<void (QString)> textureSelected;

        /// \brief A model has been saved with a name and a location
        public: static event::EventT<void (std::string, std::string)>
            saveBuildingModel;

        /// \brief A model has been completed and uploaded onto the server.
        public: static event::EventT<void ()> finishBuildingModel;

        /// \brief A new model has been started
        public: static event::EventT<void ()> newBuildingModel;

        /// \brief The current level has been changed
        public: static event::EventT<void (int)> changeBuildingLevel;

        /// \brief A new level has been added
        public: static event::EventT<void ()> addBuildingLevel;

        /// \brief A level has been deleted
        public: static event::EventT<void ()> deleteBuildingLevel;

        /// \brief Show or hide floorplan
        public: static event::EventT<void ()> showFloorplan;

        /// \brief Trigger show floorplan
        public: static event::EventT<void ()> triggerShowFloorplan;

        /// \brief Show or hide building elements
        public: static event::EventT<void ()> showElements;

        /// \brief Trigger show elements
        public: static event::EventT<void ()> triggerShowElements;

        /// \brief The levels have been changed
        public: static event::EventT<void (int, std::string)>
            updateLevelWidget;

        /// \brief The current zoom level has been changed
        public: static event::EventT<void (double)> changeBuildingEditorZoom;

        /// \brief Save the model
        public: static event::EventT<bool (std::string)> saveBuildingEditor;

        /// \brief Save the model as
        public: static event::EventT<bool (std::string)> saveAsBuildingEditor;

        /// \brief Make a new model
        public: static event::EventT<void ()> newBuildingEditor;

        /// \brief Exit the editor mode with the option to save
        public: static event::EventT<void ()> exitBuildingEditor;

        /// \brief Name was changed in the editor palette
        public: static event::EventT<void (std::string)> buildingNameChanged;
      };
    }
  }
}
#endif
