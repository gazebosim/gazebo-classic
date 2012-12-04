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

#ifndef _CORNER_GRABBER_H_
#define _CORNER_GRABBER_H_

#include "gui/qt.h"

class CornerGrabber : public QGraphicsItem
{
  public: explicit CornerGrabber(QGraphicsItem *_parent = 0);

  /// \brief Set the current mouse state
  public: void SetMouseState(int _state);

  /// \brief Retrieve the current mouse state
  public: int  GetMouseState();

  public: QPointF GetCenterPoint();

  public: void SetMouseDownX(qreal _x);

  public: void SetMouseDownY(qreal _y);

  public: qreal GetMouseDownX();

  public: qreal GetMouseDownY();

  public: void WeldCorner(CornerGrabber *_corner);

  public: CornerGrabber *GetWeldedCorner();

  public: void UnweldCorner();

  /// \brief Mouse states
  public: enum mouseStates {kMouseReleased=0, kMouseDown, kMouseMoving};

  public: virtual QRectF boundingRect() const;

  private: virtual void paint (QPainter *_painter,
    const QStyleOptionGraphicsItem *_option, QWidget *_widget);

  protected: void hoverEnterEvent (QGraphicsSceneHoverEvent *_event);

  protected: void hoverLeaveEvent (QGraphicsSceneHoverEvent *_event);

  protected: void mouseMoveEvent (QGraphicsSceneMouseEvent *_event);

  protected: void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

  protected: void mousePressEvent (QGraphicsSceneMouseEvent *_event);

  protected: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

  protected: void mouseReleaseEvent (QGraphicsSceneMouseEvent *_event);

  private: qreal mouseDownX;

  private: qreal mouseDownY;

  /// \brief the hover event handlers will toggle this between red and black
  private: QColor outterBorderColor;

  /// \brief the pen is used to paint the red/black border
  private: QPen outterBorderPen;

  private: qreal width;

  private: qreal height;

  private: int mouseButtonState;

  private: CornerGrabber* weldedCorner;
};

#endif // CORNERGRABBER_H
