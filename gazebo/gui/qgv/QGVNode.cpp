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

#include <QGVNode.h>
#include <QGVCore.h>
#include <QGVScene.h>
#include <QGVGraphPrivate.h>
#include <QGVNodePrivate.h>
#include <QDebug>
#include <QPainter>
#include <iostream>
QGVNode::QGVNode(QGVNodePrivate *node, QGVScene *qgvscene): _scene(qgvscene),
    _node(node)
{
  this->setFlag(QGraphicsItem::ItemIsSelectable, true);
//  this->setFlag(QGraphicsItem::ItemIsMovable, true);
}

QGVNode::~QGVNode()
{
    _scene->removeItem(this);
    delete _node;
}

QString QGVNode::label() const
{
    return getAttribute("label");
}

void QGVNode::setLabel(const QString &_label)
{
    setAttribute("label", _label);
}

QRectF QGVNode::boundingRect() const
{
    return _path.boundingRect();
}

void QGVNode::paint(QPainter * painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
    painter->save();

    painter->setPen(_pen);

    if (isSelected())
    {
        QBrush tbrush(_brush);
        tbrush.setColor(tbrush.color().darker(120));
        painter->setBrush(tbrush);
    }
    else
        painter->setBrush(_brush);

    painter->drawPath(_path);

    painter->setPen(QGVCore::toColor(getAttribute("labelfontcolor")));

    const QRectF rect = boundingRect();
    if (_icon.isNull())
    {
        painter->drawText(rect, Qt::AlignCenter , QGVNode::label());
    }
    else
    {
        painter->drawText(rect.adjusted(0,0,0, -rect.height()*2/3),
            Qt::AlignCenter , QGVNode::label());

        const QRectF img_rect = rect.adjusted(0, rect.height()/3,0, 0);
        QImage img = _icon.scaled(img_rect.size().toSize(),
            Qt::KeepAspectRatio, Qt::SmoothTransformation);
        painter->drawImage(img_rect.topLeft() + QPointF((img_rect.width()
            - img.rect().width())/2, 0), img);
    }
    painter->restore();
}

void QGVNode::setAttribute(const QString &name, const QString &value)
{
    agsafeset(_node->node(), name.toLocal8Bit().data(),
        value.toLocal8Bit().data(), (char *)"");
}

QString QGVNode::getAttribute(const QString &name) const
{
    char* value = agget(_node->node(), name.toLocal8Bit().data());
    if (value)
        return value;
    return QString();
}

void QGVNode::setIcon(const QImage &icon)
{
    _icon = icon;
}

void QGVNode::updateLayout()
{
    prepareGeometryChange();
    qreal width = ND_width(_node->node())*DotDefaultDPI;
    qreal height = ND_height(_node->node())*DotDefaultDPI;

    // Node Position (center)
    qreal gheight = QGVCore::graphHeight(_scene->_graph->graph());
    setPos(QGVCore::centerToOrigin(QGVCore::toPoint(ND_coord(_node->node()),
        gheight), width, height));

    // Node on top
    setZValue(1);

    // Node path
    _path = QGVCore::toPath(ND_shape(_node->node())->name,
        static_cast<polygon_t *>(ND_shape_info(_node->node())), width, height);
    _pen.setWidth(1);

    _brush.setStyle(QGVCore::toBrushStyle(getAttribute("style")));
    _brush.setColor(QGVCore::toColor(getAttribute("fillcolor")));
    _pen.setColor(QGVCore::toColor(getAttribute("color")));

    setToolTip(getAttribute("tooltip"));
}
