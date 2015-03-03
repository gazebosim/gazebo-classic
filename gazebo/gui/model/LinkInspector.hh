/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _LINK_INSPECTOR_HH_
#define _LINK_INSPECTOR_HH_

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class LinkConfig;
    class VisualConfig;
    class CollisionConfig;

    class LinkInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: LinkInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~LinkInspector();

      /// \brief Set the name of the part.
      /// \param[in] Name to set the part to.
      public: void SetName(const std::string &_name);

      /// \brief Get the name of the part.
      /// \return Name of the part.
      public: std::string GetName() const;

      /// \brief Get configurations of the link.
      /// \return Tab widget with link configurations.
      public: LinkConfig *GetLinkConfig() const;

      /// \brief Get visual configurations of the part.
      /// \return Tab widget with visual configurations.
      public: VisualConfig *GetVisualConfig() const;

      /// \brief Get collision configurations of the part.
      /// \return Tab widget with visual configurations.
      public: CollisionConfig *GetCollisionConfig() const;

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Set the item name.
      /// \param[in] _name Name to set to.
      // public: void SetName(const std::string &_name);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt signal emitted to indicate that changes should be applied
      /// and the inspector closed.
      Q_SIGNALS: void Accepted();

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

      /// \brief Widget with configurable link properties.
      private: LinkConfig *linkConfig;

      /// \brief Widget with configurable visual properties.
      private: VisualConfig *visualConfig;

      /// \brief Widget with configurable collision properties.
      private: CollisionConfig *collisionConfig;
    };
    /// \}
  }
}

#endif
