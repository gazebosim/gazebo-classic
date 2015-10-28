/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _EDIT_MODEL_WIDGET_HH_
#define _EDIT_MODEL_WIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

class QTreeWidget;

namespace gazebo
{
  namespace gui
  {
    class GZ_GUI_VISIBLE ModelEditWidget : public QWidget
    {
      Q_OBJECT
      public: ModelEditWidget(QWidget *_parent = 0);
      public: virtual ~ModelEditWidget();

      protected: void closeEvent(QCloseEvent *_event);
      protected: void showEvent(QShowEvent *_event);
      private: QTreeWidget *treeWidget;
    };

    class GZ_GUI_VISIBLE ModelPropertyWidget : public QWidget
    {
      Q_OBJECT
      public: ModelPropertyWidget(QWidget *_parent = 0);
      public: virtual ~ModelPropertyWidget();

      private: QLineEdit *nameEdit;
      private: QLineEdit *xEdit, *yEdit, *zEdit;
      private: QLineEdit *rollEdit, *pitchEdit, *yawEdit;
      private: QGroupBox *originBox;
      private: QCheckBox *staticCheck;
    };
  }
}
#endif

