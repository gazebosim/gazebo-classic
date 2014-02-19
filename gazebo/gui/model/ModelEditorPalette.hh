/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _MODEL_EDITOR_PALETTE_HH_
#define _MODEL_EDITOR_PALETTE_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class ModelEditorPalettePrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelEditorPalette ModelEditorPalette.hh
    /// \brief A palette of items which can be added to the editor.
    class ModelEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ModelEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ModelEditorPalette();

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPress(const common::KeyEvent &_event);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);

      /// \brief Qt callback when cylinder button is clicked.
      private slots: void OnCylinder();

      /// \brief Qt callback when sphere button is clicked.
      private slots: void OnSphere();

      /// \brief Qt callback when box button is clicked.
      private slots: void OnBox();

      /// \brief Qt callback when custom button is clicked.
      private slots: void OnCustom();

      /// \brief Qt callback when fixed joint button is clicked.
      private slots: void OnFixedJoint();

      /// \brief Qt callback when hinge joint button is clicked.
      private slots: void OnHingeJoint();

      /// \brief Qt callback when hinge2 joint button is clicked.
      private slots: void OnHinge2Joint();

      /// \brief Qt callback when slider joint button is clicked.
      private slots: void OnSliderJoint();

      /// \brief Qt callback when screw joint button is clicked.
      private slots: void OnScrewJoint();

      /// \brief Qt callback when universal joint button is clicked.
      private slots: void OnUniversalJoint();

      /// \brief Qt callback when universal joint button is clicked.
      private slots: void OnBallJoint();

      /// \brief Qt callback when a joint has been added.
      private slots: void OnJointAdded();

      /// \brief Qt callback when a part has been added.
      private slots: void OnPartAdded();

      /// \brief Qt callback when the model is to be made static.
      private slots: void OnStatic();

      /// \brief Qt callback when the model is allowed to auto disable at rest.
      private slots: void OnAutoDisable();

      /// \brief Qt callback when the model is to be saved.
      private slots: void OnSave();

      /// \brief Qt callback when the model is to be discarded.
      private slots: void OnDiscard();

      /// \brief Qt callback when model editing is complete.
      private slots: void OnDone();

      /// \internal
      /// \brief Pointer to private data.
      private: ModelEditorPalettePrivate *dataPtr;
    };
  }
}
#endif
