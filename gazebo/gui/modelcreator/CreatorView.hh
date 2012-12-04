/*
 * Copyright 2012 Nate Koenig
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
#ifndef _CREATOR_SCENE_HH_
#define _CREATOR_SCENE_HH_

#include <string>
#include "gui/qt.h"

#include "common/MouseEvent.hh"
#include "common/Event.hh"

class SelectableLineSegment;

namespace gazebo
{
  namespace gui
  {
    class CreatorView : public QGraphicsView
    {
      Q_OBJECT

      public: CreatorView(QWidget *_parent = 0);

      public: ~CreatorView();

      public: enum modelTypes {None, Wall, Window, Door, Stairs};

      protected: void mousePressEvent(QMouseEvent *_event);

      protected: void mouseReleaseEvent(QMouseEvent *_event);

      protected: void mouseMoveEvent(QMouseEvent *_event);

      protected: void mouseDoubleClickEvent(QMouseEvent *_event);

      private: void DrawLines(QPoint _pos);

      private: int drawMode;

      private: bool drawInProgress;

      private: std::vector<SelectableLineSegment*> lineList;

      private: SelectableLineSegment *currentLine;

      private: QPoint lastLineCornerPos;

      private: QPoint lastMousePos;
    };
  }
}

#endif
