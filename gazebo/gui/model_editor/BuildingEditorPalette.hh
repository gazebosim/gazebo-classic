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

#include "gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class BuildingEditorPalette : public QWidget
    {
      Q_OBJECT

      public: BuildingEditorPalette(QWidget *_parent = 0);
      public: ~BuildingEditorPalette();

      private: std::string modelName;

      /// \brief On discard callback.
      private slots: void OnDiscard();

      /// \brief On save callback.
      private slots: void OnSave();

      /// \brief On finish callback.
      private slots: void OnFinish();


    };
  }
}

#endif
