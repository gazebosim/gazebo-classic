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

#ifndef _WINDOW_ITEM_HH_
#define _WINDOW_ITEM_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RectItem;
    class BuildingItem;
    class WindowDoorInspectorDialog;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class WindowItem WindowItem.hh
    /// \brief 2D representation of a window.
    class GZ_GUI_BUILDING_VISIBLE WindowItem : public RectItem, public BuildingItem
    {
        Q_OBJECT

        /// \brief Constructor
        public: WindowItem();

        /// \brief Destructor
        public: ~WindowItem();

        // Documentation inherited
        public: virtual QVector3D GetSize() const;

        // Documentation inherited
        public: virtual QVector3D GetScenePosition() const;

        // Documentation inherited
        public: virtual double GetSceneRotation() const;

        // Documentation inherited
        private: virtual void paint(QPainter *_painter,
            const QStyleOptionGraphicsItem *_option, QWidget *_widget);

        // Documentation inherited
        private: void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event);

        // Documentation inherited
        private slots: void OnApply();

        // Documentation inherited
        private slots: void OnOpenInspector();

        // Documentation inherited
        private slots: void OnDeleteItem();

        /// \brief Emit window changed Qt signals.
        public: void WindowChanged();

        /// \brief Depth of the window item in pixels.
        private: double windowDepth;

        /// \brief Height of the window item in pixels.
        private: double windowHeight;

        /// \brief Width of the window item in pixels.
        private: double windowWidth;

        /// \brief Side bar of the window item in pixels.
        private: double windowSideBar;

        /// \brief Scene position of the window item in pixel coordinates.
        private: QPointF windowPos;

        /// \brief Elevation of the window item in pixels.
        private: double windowElevation;

        /// \brief Scale for converting pixels to metric units.
        private: double scale;

        /// \brief Inspector for configuring the window item.
        private: WindowDoorInspectorDialog *inspector;
    };
    /// \}
  }
}
#endif
