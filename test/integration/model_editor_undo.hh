/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_TEST_INTEGRATION_MODELEDITORUNDO_HH_
#define _GAZEBO_TEST_INTEGRATION_MODELEDITORUNDO_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for undo / redo link insertion in the model editor.
class ModelEditorUndoTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Test undo/redo link insertion using the mouse.
  private slots: void LinkInsertionByMouse();

  /// \brief Test undo/redo link deletion via the right-click context menu.
  private slots: void LinkDeletionByContextMenu();

  /// \brief Test undo/redo nested model insertion using the mouse.
  private slots: void NestedModelInsertionByMouse();

  /// \brief Test undo/redo nested model deletion via the right-click context
  /// menu, as well as the model's joints.
  private slots: void NestedModelDeletionByContextMenu();

  /// \brief Test undo/redo joint insertion via creation dialog.
  private slots: void JointInsertionByDialog();

  /// \brief Test undo/redo model plugin insertion.
  private slots: void ModelPluginInsertion();

  /// \brief Test undo/redo model plugin deletion via the right-click context
  /// menu.
  private slots: void ModelPluginDeletionByContextMenu();

  /// \brief Test undo/redo link translation.
  private slots: void LinkTranslation();

  /// \brief Test undo/redo nested model align.
  private slots: void NestedModelAlign();

  /// \brief Test undo/redo link scaling.
  private slots: void LinkScaling();

  /// \brief Helper callback to trigger the delete action on the context
  /// menu after the menu, which is modal, has been opened.
  private slots: void TriggerDelete();
};

#endif

