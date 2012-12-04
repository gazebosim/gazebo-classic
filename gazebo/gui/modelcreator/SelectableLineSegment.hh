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

#ifndef _SELECTABLE_LINESEGMENT_H
#define _SELECTABLE_LINESEGMENT_H

#include <gui/qt.h>

class CornerGrabber;

class SelectableLineSegment : public QGraphicsPolygonItem
{
  public: SelectableLineSegment(QPointF _start, QPointF _end);

  public: ~SelectableLineSegment();

  public: void SetCornerPosition(QPointF _position, int _cornerIndex);

  public: void TranslateCorner(QPointF _translation, int _cornerIndex);

  public: void ConnectLine(SelectableLineSegment *_line);

  public: void DisconnectLine(SelectableLineSegment *_line);

  public: SelectableLineSegment *GetAdjacentLine(int _index);

  public: CornerGrabber *GetCorner(int _index);

  public: bool HasCorner(CornerGrabber *_corner);

  private: void UpdateCornerPositions();

  private: void CreateCustomPath(QPointF _mouseLocation, CornerGrabber*);

  private: void EnclosePath(qreal _lineStartX,qreal _lineStartY,
      qreal _lineEndX, qreal _lineEndY);

  protected:  void paint (QPainter *painter,
      const QStyleOptionGraphicsItem *_option, QWidget *_widget);

  protected: void hoverEnterEvent ( QGraphicsSceneHoverEvent *_event);

  protected: void hoverLeaveEvent ( QGraphicsSceneHoverEvent *_event);

  protected: void mouseMoveEvent ( QGraphicsSceneMouseEvent *_event);

  protected: void mousePressEvent (QGraphicsSceneMouseEvent *_event);

  protected: void mouseReleaseEvent (QGraphicsSceneMouseEvent *_event);

  protected: void mouseMoveEvent(QGraphicsSceneDragDropEvent *event);

  protected: void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

  protected: bool sceneEventFilter(QGraphicsItem * watched,
    QEvent *_event) ;

  /// \brief the hover event handler will toggle this between red and black
  private: QColor outterBorderColor;

  /// \brief the pen is used to paint the red/black border
  private: QPen pen;

  private: QPointF location;

  private: QPointF dragStart;

  private: int gridSpace;

  private: qreal lineLength;

  private: int xCornerGrabBuffer;

  private: int yCornerGrabBuffer;

//  private: qreal graphicsItemBoundingBoxWidth;

  private: QPointF lineEnd0;

  private: QPointF lineEnd1;

  private: bool cornerGrabbed;

  // 0,1  - starting at x=0,y=0
  private: CornerGrabber*  corners[2];

  private: QPolygonF selectRegion;

  private: SelectableLineSegment *adjacentLineSegments[2];

};

#endif
