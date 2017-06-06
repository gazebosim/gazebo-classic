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
#include "QGVScene.h"
// The following include allows the automoc to detect, that it must moc this
// class
//#include "moc_QGVScene.cpp"
#include <QDebug>

#include <QGVNode.h>
#include <QGVEdge.h>
#include <QGVSubGraph.h>

#include <QGVCore.h>
#include <QGVGraphPrivate.h>
#include <QGVGvcPrivate.h>
#include <QGVEdgePrivate.h>
#include <QGVNodePrivate.h>
#include <iostream>

QGVScene::QGVScene(const QString &name, QObject *_parent)
    : QGraphicsScene(_parent)
{
    _context = new QGVGvcPrivate(gvContext());
    _graph = new QGVGraphPrivate(agopen(name.toLocal8Bit().data(), Agdirected, NULL));
    // setGraphAttribute("fontname", QFont().family());
    this->init = false;
}

QGVScene::~QGVScene()
{
    gvFreeLayout(_context->context(), _graph->graph());
    agclose(_graph->graph());
    gvFreeContext(_context->context());
    delete _graph;
    delete _context;
}

void QGVScene::setGraphAttribute(const QString &name, const QString &value)
{
  agattr(_graph->graph(), AGRAPH, name.toLocal8Bit().data(), value.toLocal8Bit().data());
}

void QGVScene::setNodeAttribute(const QString &name, const QString &value)
{
  agattr(_graph->graph(), AGNODE, name.toLocal8Bit().data(),
      value.toLocal8Bit().data());
}

void QGVScene::setEdgeAttribute(const QString &name, const QString &value)
{
  agattr(_graph->graph(), AGEDGE, name.toLocal8Bit().data(),
      value.toLocal8Bit().data());
}

QGVNode *QGVScene::addNode(const QString &label)
{
  Agnode_t *node = agnode(_graph->graph(), label.toLocal8Bit().data(), true);
  if (node == NULL)
  {
      qWarning()<<"Invalid node :"<<label;
      return 0;
  }

  QGVNode *item = new QGVNode(new QGVNodePrivate(node), this);
  item->setLabel(label);
  addItem(item);
  _nodes.insert(label, item);
  return item;
}

QGVEdge *QGVScene::addEdge(QGVNode *source, QGVNode *target,
    const QString &label)
{
  Agedge_t* edge = agedge(_graph->graph(), source->_node->node(),
      target->_node->node(), NULL, true);
  if (edge == NULL)
  {
      qWarning() << "Invalid egde :" << label;
      return 0;
  }

  QGVEdge *item = new QGVEdge(new QGVEdgePrivate(edge), this);

  item->setSource(source->label());
  item->setTarget(target->label());
  addItem(item);

  _edges.insert(label, item);
  return item;
}


QGVEdge *QGVScene::addEdge(const QString &source, const QString &target,
    const QString &label)
{
  if (_nodes.contains(source) && _nodes.contains(target))
    return this->addEdge(_nodes[source], _nodes[target], label);
  return NULL;
}


QGVSubGraph *QGVScene::addSubGraph(const QString &name, bool cluster)
{
  Agraph_t* sgraph;
  if (cluster)
  {
    sgraph = agsubg(_graph->graph(),
        ("cluster_" + name).toLocal8Bit().data(), true);
  }
  else
    sgraph = agsubg(_graph->graph(), name.toLocal8Bit().data(), true);

  if (sgraph == NULL)
  {
      qWarning()<<"Invalid subGraph :"<<name;
      return 0;
  }

  QGVSubGraph *item = new QGVSubGraph(new QGVGraphPrivate(sgraph), this);
  addItem(item);
  _subGraphs.insert(name, item);
  return item;
}

void QGVScene::removeNode(const QString &label)
{
  if (_nodes.contains(label))
  {
    agdelete(_graph->graph(), _nodes[label]->_node->node());
    removeItem(_nodes[label]);
    _nodes.remove(label);
  }

  QList<QString> toRemove;

  for (auto key : _edges.toStdMap())
  {
    QGVEdge *e = key.second;
    if (e->source() == label || e->target() == label)
      toRemove.append(key.first);
  }
  for (auto e : toRemove)
    this->removeEdge(e);
}

void QGVScene::removeEdge(const QString &_label)
{
  if (_edges.contains(_label))
  {
    agdelete(_graph->graph(), _edges[_label]->_edge->edge());
    removeItem(_edges[_label]);
    _edges.remove(_label);
  }
}

bool QGVScene::hasNode(const QString &_name)
{
  return _nodes.contains(_name);
}

QGVNode *QGVScene::getNode(const QString &_name)
{
  if (this->hasNode(_name))
    return _nodes[_name];
  else
    return NULL;
}

bool QGVScene::hasEdge(const QString &_label)
{
  return _edges.contains(_label);
}

QGVEdge *QGVScene::getEdge(const QString &_label)
{
  if (this->hasEdge(_label))
    return _edges[_label];
  else
    return NULL;
}

int QGVScene::nodeCount() const
{
  return _nodes.size();
}

int QGVScene::edgeCount() const
{
  return _edges.size();
}

void QGVScene::setRootNode(QGVNode *node)
{
    Q_ASSERT(_nodes.contains(node->label()));
    agset(_graph->graph(), (char *)"root", node->label().toLocal8Bit().data());
}

void QGVScene::loadLayout(const QString &text)
{
  _graph->setGraph(QGVCore::agmemread2(text.toLocal8Bit().constData()));

  if (gvLayout(_context->context(), _graph->graph(), "dot") != 0)
  {
    qCritical() << "Layout render error" << agerrors() <<
        QString::fromLocal8Bit(aglasterr());
    return;
  }

  // Debug output
  // gvRenderFilename(_context->context(), _graph->graph(), "png",
  //    "debug.png");

  // Read nodes and edges
  for (Agnode_t* node = agfstnode(_graph->graph()); node != NULL;
      node = agnxtnode(_graph->graph(), node))
  {
    QGVNode *inode = new QGVNode(new QGVNodePrivate(node), this);
    inode->updateLayout();
    addItem(inode);
    for (Agedge_t* edge = agfstout(_graph->graph(), node); edge != NULL;
        edge = agnxtout(_graph->graph(), edge))
    {
      QGVEdge *iedge = new QGVEdge(new QGVEdgePrivate(edge), this);
      iedge->updateLayout();
      addItem(iedge);
    }
  }
  update();
}

void QGVScene::applyLayout()
{
  if (_nodes.empty())
    return;

  gvFreeLayout(_context->context(), _graph->graph());
  if (gvLayout(_context->context(), _graph->graph(), "dot") != 0)
  {
    qCritical()<<"Layout render error" <<
        agerrors()<<QString::fromLocal8Bit(aglasterr());
    return;
  }

  // Debug output
  // gvRenderFilename(_context->context(), _graph->graph(), "canon",
  //    "debug.dot");
  // gvRenderFilename(_context->context(), _graph->graph(), "png",
  //    "debug.png");

  //Update items layout
  foreach(QGVNode* node, _nodes)
      node->updateLayout();

  foreach(QGVEdge* edge, _edges)
      edge->updateLayout();

  foreach(QGVSubGraph* sgraph, _subGraphs)
      sgraph->updateLayout();

  // Graph label
  textlabel_t *xlabel = GD_label(_graph->graph());
  if (xlabel)
  {
    QGraphicsTextItem *item = addText(xlabel->text);
    item->setPos(QGVCore::centerToOrigin(QGVCore::toPoint(xlabel->pos,
        QGVCore::graphHeight(_graph->graph())), xlabel->dimen.x, -4));
  }

  update();
}

void QGVScene::clearLayout()
{
  gvFreeLayout(_context->context(), _graph->graph());
}

void QGVScene::clear()
{
  gvFreeLayout(_context->context(), _graph->graph());
  _nodes.clear();
  _edges.clear();
  _subGraphs.clear();
  QGraphicsScene::clear();
}

#include <QGraphicsSceneContextMenuEvent>
void QGVScene::contextMenuEvent(
    QGraphicsSceneContextMenuEvent *_contextMenuEvent)
{
  QGraphicsItem *item = itemAt(_contextMenuEvent->scenePos(), QTransform());
  if (item)
  {
    item->setSelected(true);
    if (item->type() == QGVNode::Type)
      emit nodeContextMenu(qgraphicsitem_cast<QGVNode*>(item));
    else if (item->type() == QGVEdge::Type)
      emit edgeContextMenu(qgraphicsitem_cast<QGVEdge*>(item));
    else if (item->type() == QGVSubGraph::Type)
      emit subGraphContextMenu(qgraphicsitem_cast<QGVSubGraph*>(item));
    else
      emit graphContextMenuEvent();
  }
  QGraphicsScene::contextMenuEvent(_contextMenuEvent);
}

void QGVScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
  QGraphicsItem *item = itemAt(mouseEvent->scenePos(), QTransform());
  if (item)
  {
    if (item->type() == QGVNode::Type)
      emit nodeDoubleClick(qgraphicsitem_cast<QGVNode*>(item));
    else if (item->type() == QGVEdge::Type)
      emit edgeDoubleClick(qgraphicsitem_cast<QGVEdge*>(item));
    else if (item->type() == QGVSubGraph::Type)
      emit subGraphDoubleClick(qgraphicsitem_cast<QGVSubGraph*>(item));
  }
  QGraphicsScene::mouseDoubleClickEvent(mouseEvent);
}

#include <QVarLengthArray>
#include <QPainter>
void QGVScene::drawBackground(QPainter * painter, const QRectF & rect)
{
  const int gridSize = 25;

  const qreal left = int(rect.left()) - (int(rect.left()) % gridSize);
  const qreal top = int(rect.top()) - (int(rect.top()) % gridSize);

  QVarLengthArray<QLineF, 100> lines;

  for (qreal x = left; x < rect.right(); x += gridSize)
      lines.append(QLineF(x, rect.top(), x, rect.bottom()));
  for (qreal y = top; y < rect.bottom(); y += gridSize)
      lines.append(QLineF(rect.left(), y, rect.right(), y));

  painter->setRenderHint(QPainter::Antialiasing, false);

  painter->setPen(QColor(Qt::lightGray).lighter(110));
  painter->drawLines(lines.data(), lines.size());
  painter->setPen(Qt::black);
  // painter->drawRect(sceneRect());
}
