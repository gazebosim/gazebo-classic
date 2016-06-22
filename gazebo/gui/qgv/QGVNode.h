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
#ifndef QGVNODE_H
#define QGVNODE_H

#ifndef _MSC_VER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
#include <QGraphicsItem>
#include <QPen>
#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif

#include "qgv.h"

class QGVEdge;
class QGVScene;
class QGVNodePrivate;

/**
 * @brief Node item
 *
 */
class QGVCORE_EXPORT QGVNode : public QGraphicsItem
{
  public:
    ~QGVNode();

    QString label() const;
    void setLabel(const QString &label);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
        QWidget *widget = 0);
    void setAttribute(const QString &label, const QString &value);
    QString getAttribute(const QString &name) const;

    void setIcon(const QImage &icon);

    enum { Type = UserType + 2 };
    int type() const
    {
        return Type;
    }

  private:
    friend class QGVScene;
    friend class QGVSubGraph;
    void updateLayout();
    QGVNode(QGVNodePrivate* node, QGVScene *scene);

    // Not implemented in QGVNode.cpp
//    QPainterPath makeShape(Agnode_t* node) const;
//    QPolygonF makeShapeHelper(Agnode_t* node) const;

    QPainterPath _path;
    QPen _pen;
    QBrush _brush;
    QImage _icon;

    QGVScene *_scene;
    QGVNodePrivate* _node;
};


#endif
