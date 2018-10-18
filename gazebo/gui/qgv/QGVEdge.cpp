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

#include <QGVEdge.h>
#include <QGVCore.h>
#include <QGVScene.h>
#include <QGVGraphPrivate.h>
#include <QGVEdgePrivate.h>
#include <QDebug>
#include <QPainter>

QGVEdge::QGVEdge(QGVEdgePrivate *edge, QGVScene *qgvscene)
    :  _scene(qgvscene), _edge(edge)
{
//    setFlag(QGraphicsItem::ItemIsSelectable, true);
}

QGVEdge::~QGVEdge()
{
    _scene->removeItem(this);
    delete _edge;
}

QString QGVEdge::label() const
{
    return getAttribute("xlabel");
}

QRectF QGVEdge::boundingRect() const
{
    return _path.boundingRect() | _head_arrow.boundingRect() | _tail_arrow.boundingRect() | _label_rect;
}

QPainterPath QGVEdge::shape() const
{
    QPainterPathStroker ps;
    ps.setCapStyle(_pen.capStyle());
    ps.setWidth(_pen.widthF() + 10);
    ps.setJoinStyle(_pen.joinStyle());
    ps.setMiterLimit(_pen.miterLimit());
    return ps.createStroke(_path);
}

void QGVEdge::setLabel(const QString &_xlabel)
{
    setAttribute("xlabel", _xlabel);
}

void QGVEdge::setSource(const QString &_source)
{
  this->sourceNode = _source;
}

void QGVEdge::setTarget(const QString &_target)
{
  this->targetNode = _target;
}

QString QGVEdge::source() const
{
  return sourceNode;
}

QString QGVEdge::target() const
{
  return targetNode;
}

void QGVEdge::paint(QPainter * painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
    painter->save();

    if(isSelected())
    {
        QPen tpen(_pen);
        tpen.setColor(_pen.color().darker(120));
        tpen.setStyle(Qt::DotLine);
        painter->setPen(tpen);
    }
    else
        painter->setPen(_pen);


    painter->drawPath(_path);

    /*
    QRectF pp = _path.controlPointRect();
    if(pp.width() < pp.height())
    {
        painter->save();
        painter->translate(_label_rect.topLeft());
        painter->rotate(90);
        painter->drawText(QRectF(QPointF(0, -_label_rect.width()),
            _label_rect.size()), Qt::AlignCenter, _label);
        painter->restore();
    }
    else
    */
    painter->drawText(_label_rect, Qt::AlignCenter, _label);

    painter->setBrush(QBrush(_pen.color(), Qt::SolidPattern));
    painter->drawPolygon(_head_arrow);
    painter->drawPolygon(_tail_arrow);
    painter->restore();
}

void QGVEdge::setAttribute(const QString &name, const QString &value)
{
    agsafeset(_edge->edge(), name.toLocal8Bit().data(),
        value.toLocal8Bit().data(), (char *)"");
}

QString QGVEdge::getAttribute(const QString &name) const
{
    char* value = agget(_edge->edge(), name.toLocal8Bit().data());
    if(value)
        return value;
    return QString();
}

void QGVEdge::updateLayout()
{
    prepareGeometryChange();

    qreal gheight = QGVCore::graphHeight(_scene->_graph->graph());

    const splines* spl = ED_spl(_edge->edge());
    _path = QGVCore::toPath(spl, gheight);


    //Edge arrows
    if((spl->list != 0) && (spl->list->size%3 == 1))
    {
        if(spl->list->sflag)
        {
            _tail_arrow = toArrow(
                QLineF(QGVCore::toPoint(spl->list->list[0], gheight),
                QGVCore::toPoint(spl->list->sp, gheight)));
        }

        if(spl->list->eflag)
        {
            _head_arrow = toArrow(
                QLineF(QGVCore::toPoint(spl->list->list[spl->list->size-1],
                gheight), QGVCore::toPoint(spl->list->ep, gheight)));
        }
    }

    _pen.setWidth(1);
    _pen.setColor(QGVCore::toColor(getAttribute("color")));
    _pen.setStyle(QGVCore::toPenStyle(getAttribute("style")));

    // Edge label
    textlabel_t *xlabel = ED_xlabel(_edge->edge());
    if(xlabel)
    {
        _label = xlabel->text;
        _label_rect.setSize(QSize(xlabel->dimen.x, xlabel->dimen.y));
        _label_rect.moveCenter(QGVCore::toPoint(xlabel->pos,
            QGVCore::graphHeight(_scene->_graph->graph())));
    }

    setToolTip(getAttribute("tooltip"));
}

QPolygonF QGVEdge::toArrow(const QLineF &line) const
{
    QLineF n = line.normalVector();
    QPointF o(n.dx() / 3.0, n.dy() / 3.0);

    // Only support normal arrow type
    QPolygonF polygon;
    polygon.append(line.p1() + o);
    polygon.append(line.p2());
    polygon.append(line.p1() - o);

    return polygon;
}

QPolygonF QGVEdge::toBox(const QLineF &line) const
{
    QLineF n = line.normalVector();
    QPointF o(n.dx() * 0.5, n.dy() * 0.5);

    QPolygonF polygon;
    polygon.append(line.p1() + o);
    polygon.append(line.p2() + o);
    polygon.append(line.p2() - o);
    polygon.append(line.p1() - o);
    return polygon;
}
