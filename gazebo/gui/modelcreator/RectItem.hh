
#ifndef _RECT_ITEM_H_
#define _RECT_ITEM_H_

#include "gui/qt.h"

class CornerGrabber;

class RectItem : public QGraphicsItem
{
    public: RectItem();

    public: ~RectItem();

    public: QGraphicsTextItem text;

    public: void SetGridSpace(int _space);

    private: void UpdateCornerPositions();

    private: void AdjustSize(int _x, int _y);

    private: virtual QRectF boundingRect() const;

    private: virtual void paint (QPainter *_painter,
        const QStyleOptionGraphicsItem *_option, QWidget *_widget);

    private: virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *_event);

    private: virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent *_event);

    private: virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *_event);

    private: virtual void mousePressEvent(QGraphicsSceneMouseEvent *_event);

    private: virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *_event);

    private: virtual void mouseMoveEvent(QGraphicsSceneDragDropEvent *_event);

    private: virtual void mousePressEvent(QGraphicsSceneDragDropEvent *_event);

    private: virtual bool sceneEventFilter(QGraphicsItem *_watched,
        QEvent *_event);

    private: QColor outterBorderColor;

    private: QPen outterBorderPen;

    private: QPointF location;

    private: QPointF dragStart;

    private: int gridSpace;

    private: qreal width;

    private: qreal height;

    private: QPointF cornerDragStart;

    private: int xCornerGrabBuffer;

    private: int yCornerGrabBuffer;

    private: qreal drawingWidth;

    private: qreal drawingHeight;

    private: qreal drawingOriginX;

    private: qreal drawingOriginY;

    /// \brief Corners of the rectangle, going clockwise with 0 being top left
    CornerGrabber*  corners[4];
};

#endif // STATEBOX_H
