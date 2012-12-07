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

#include "PolylineItem.hh"
#include "WallItem.hh"
#include "WallInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallItem::WallItem(QPointF _start, QPointF _end)
    : PolylineItem(_start, _end)
{
}

WallItem::~WallItem()
{
}

/////////////////////////////////////////////////
void WallItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  WallInspectorDialog dialog;
//  dialog.SetWidth(this->GetWidth());
//  dialog.SetHeight(this->GetHeight());
  if (dialog.exec() == QDialog::Accepted)
  {
//    this->SetSize(QSize(dialog.GetWidth(), dialog.GetHeight()));
  }
  _event->setAccepted(true);
}
