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

#ifndef _POLYLINE_ITEM_H
#define _POLYLINE_ITEM_H

#include <gui/qt.h>

class CornerGrabber;

namespace gazebo
{
  namespace gui
  {

  class PolylineItem : public QGraphicsPolygonItem
  {
    public: PolylineItem(QPointF _start, QPointF _end);

    public: ~PolylineItem();

    public: void AddPoint(QPointF F);

    private: std::vector<CornerGrabber*> corners;

    private: int cornerWidth;

    private: int cornerHeight;
  };
}

#endif
