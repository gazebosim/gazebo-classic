/***************************************************************
QGVCore
Copyright (c) 2014, Bergont Nicolas, All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3.0 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library.
***************************************************************/

#ifndef QGVEDGE_H
#define QGVEDGE_H

#include <qgv.h>
#include <QGraphicsItem>
#include <QPen>

class QGVNode;
class QGVScene;
class QGVEdgePrivate;

/**
 * @brief Edge item
 *
 */
class QGVCORE_EXPORT QGVEdge : public QGraphicsItem
{
  public:
    ~QGVEdge();

    QString label() const;
    QRectF boundingRect() const;
    QPainterPath shape() const;

    QString source() const;
    QString target() const;

    void setLabel(const QString &label);

    void setSource(const QString &_source);
    void setTarget(const QString &_target);

    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option,
        QWidget * widget = 0);

    void setAttribute(const QString &name, const QString &value);
    QString getAttribute(const QString &name) const;

    void updateLayout();

    enum { Type = UserType + 3 };
    int type() const
    {
        return Type;
    }

  private:
    QGVEdge(QGVEdgePrivate *edge, QGVScene *scene);

    QPolygonF toArrow(const QLineF &normal) const;
    QPolygonF toBox(const QLineF &normal) const;

    friend class QGVScene;
    // friend class QGVSubGraph;

    QGVScene *_scene;
    QGVEdgePrivate* _edge;

    QPainterPath _path;
    QPen _pen;
    QPolygonF _head_arrow;
    QPolygonF _tail_arrow;

    QString _label;
    QRectF _label_rect;

    QString sourceNode;
    QString targetNode;
};

#endif
