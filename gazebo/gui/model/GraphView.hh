/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MODEL_GRAPHVIEW_HH_
#define GAZEBO_GUI_MODEL_GRAPHVIEW_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class GraphView GraphView.hh
    /// \brief A view for the graph scene
    class GraphView : public QGraphicsView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent Widget.
      public: GraphView(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~GraphView() = default;

      /// \brief Qt resize event received when the parent widget changes size.
      /// \param[in] _event Qt resize event
      private: void resizeEvent(QResizeEvent *_event);

      /// \brief Qt event received when the editor view is being scrolled.
      /// \param[in] _dx Change in X position of the scrollbar.
      /// \param[in] _dx Change in Y position of the scrollbar.
      private: void scrollContentsBy(int _dx, int _dy);

      /// \brief Qt context menu received on a mouse right click.
      /// \param[in] _event Qt context menu event.
      private: void contextMenuEvent(QContextMenuEvent *_event);

      /// \brief Qt wheel event received when the mouse wheel is being scrolled.
      /// \param[in] _event Qt wheel event.
      private: void wheelEvent(QWheelEvent *_event);

      /// \brief Qt mouse move event.
      /// \param[in] _event Qt mouse event.
      private: void mouseMoveEvent(QMouseEvent *_event);

      /// \brief Qt mouse press event.
      /// \param[in] _event Qt mouse event.
      private: void mousePressEvent(QMouseEvent *_event);

      /// \brief Qt mouse release event.
      /// \param[in] _event Qt mouse event.
      private: void mouseReleaseEvent(QMouseEvent *_event);

      /// \brief Qt mouse double click event.
      /// \param[in] _event Qt mouse event.
      private: void mouseDoubleClickEvent(QMouseEvent *_event);

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event.
      private: void keyPressEvent(QKeyEvent *_event);

      /// \brief Qt signal emitted when a context menu event is triggered.
      /// \param[in] _id Unique id of an item.
      Q_SIGNALS: void customContextMenuRequested(QString _id);

      /// \brief Qt signal emitted when an item is double clicked.
      /// \param[in] _id Unique id of an item.
      Q_SIGNALS: void itemDoubleClicked(QString _id);

      /// \brief Store which item was clicked last.
      public: QGraphicsItem *lastClickedItem = nullptr;

      /// \brief Scale (zoom level) of the editor view.
      private: double viewScale;
    };
  }
}

#endif
