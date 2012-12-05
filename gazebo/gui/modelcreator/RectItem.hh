
#ifndef _RECT_ITEM_H_
#define _RECT_ITEM_H_

#include "gui/qt.h"

class CornerGrabber;

class RectItem : public QGraphicsItem
{
    public: RectItem();

    public: virtual ~RectItem();

    public: void SetGridSpace(int _space);

    protected: void UpdateCornerPositions();

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

    protected: qreal width;

    protected: qreal height;

    protected: qreal drawingWidth;

    protected: qreal drawingHeight;

    protected: qreal drawingOriginX;

    protected: qreal drawingOriginY;

    protected: QColor outterBorderColor;

    private: QPointF location;

    private: QPointF dragStart;

    private: int gridSpace;

    private: QPointF cornerDragStart;

    private: int xCornerGrabBuffer;

    private: int yCornerGrabBuffer;

    /// \brief Corners of the rectangle, going clockwise with 0 being top left
    CornerGrabber*  corners[4];
};

#endif
