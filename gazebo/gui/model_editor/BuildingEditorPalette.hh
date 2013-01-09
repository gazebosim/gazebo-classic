/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _BUILDING_EDITOR_PALETTE_HH_
#define _BUILDING_EDITOR_PALETTE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class BuildingEditorPalette : public QWidget
    {
      Q_OBJECT

      public: BuildingEditorPalette(QWidget *_parent = 0);

      public: ~BuildingEditorPalette();

//      public: void SetEditorWidget(BuildingEditorWidget* _widget);

      /// \brief On draw wall callback.
      private slots: void OnDrawWall();

      /// \brief On import image callback.
      private slots: void OnImportImage();

      /// \brief On add window callback.
      private slots: void OnAddWindow();

      /// \brief On add door callback.
      private slots: void OnAddDoor();

      /// \brief On add stairs callback.
      private slots: void OnAddStairs();

      /// \brief On discard callback.
      private slots: void OnDiscard();

      /// \brief On save callback.
      private slots: void OnSave();

      /// \brief On done callback.
      private slots: void OnDone();

      private: QLabel *modelNameLabel;

      private: QPushButton *saveButton;

      private: std::string modelName;

      private: std::string saveLocation;

      private: bool saved;
    };
  }
}

#endif
