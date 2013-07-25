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

#ifndef _PART_INSPECTOR_HH_
#define _PART_INSPECTOR_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class PartGeneralTab;
    class PartVisualTab;

    class PartInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: PartInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~PartInspector();

      /// \brief Get general properties of the part.
      /// \return Tab widget with general properties.
      public: PartGeneralTab *GetGeneral() const;

      /// \brief Get visual properties of the part.
      /// \return Tab widget with visual properties.
      public: PartVisualTab *GetVisual() const;

      /// \brief Set the item name.
      /// \param[in] _name Name to set to.
      // public: void SetName(const std::string &_name);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Main tab widget within the part inspector.
      private: QTabWidget *tabWidget;

      /// \brief Label that displays the name of the part.
      private: QLabel* partNameLabel;

      /// \brief Tab widget with configurable general properties.
      private: PartGeneralTab *generalTab;

      /// \brief Tab widget with configurable visual properties.
      private: PartVisualTab *visualTab;
    };
    /// \}
  }
}

#endif
