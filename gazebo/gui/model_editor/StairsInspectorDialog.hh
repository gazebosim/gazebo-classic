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

#ifndef _STAIRS_INSPECTOR_DIALOG_HH_
#define _STAIRS_INSPECTOR_DIALOG_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class StairsInspectorDialog : public QDialog
    {
      Q_OBJECT

      public: StairsInspectorDialog(QWidget *_parent = 0);

      public: ~StairsInspectorDialog();

      public: QPointF GetStartPosition() const;

      public: double GetWidth() const;

      public: double GetDepth() const;

      public: double GetHeight() const;

      public: int GetSteps() const;

      public: void SetName(const std::string &_name);

      public: void SetStartPosition(const QPointF &_pos);

      public: void SetWidth(double _width);

      public: void SetDepth(double _depth);

      public: void SetHeight(double _height);

      public: void SetSteps(int _steps);

      Q_SIGNALS: void Applied();

      private slots: void OnCancel();

      private slots: void OnApply();

      private slots: void OnOK();

      private: QDoubleSpinBox *startXSpinBox;

      private: QDoubleSpinBox *startYSpinBox;

      private: QDoubleSpinBox *widthSpinBox;

      private: QDoubleSpinBox *depthSpinBox;

      private: QDoubleSpinBox *heightSpinBox;

      private: QSpinBox *stepsSpinBox;

      private: QLabel* stairsNameLabel;
   };
 }
}

#endif
