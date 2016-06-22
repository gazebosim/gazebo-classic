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

#ifndef QGVSCENE_H
#define QGVSCENE_H

#ifndef _MSC_VER
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
#include <QMap>
#include <QGraphicsScene>
#ifndef _MSC_VER
#pragma GCC diagnostic pop
#endif

#include "qgv.h"
#include "gazebo/util/system.hh"

class QGVNode;
class QGVEdge;
class QGVSubGraph;

class QGVGraphPrivate;
class QGVGvcPrivate;

/**
 * @brief GraphViz interactive scene
 *
 */
class GZ_GUI_VISIBLE QGVScene : public QGraphicsScene
{
    Q_OBJECT
  public:

    explicit QGVScene(const QString &name, QObject *parent = 0);
    ~QGVScene();

    void setGraphAttribute(const QString &name, const QString &value);
    void setNodeAttribute(const QString &name, const QString &value);
    void setEdgeAttribute(const QString &name, const QString &value);

    QGVNode* addNode(const QString& label);
    QGVEdge* addEdge(QGVNode* source, QGVNode* target, const QString& label);
    QGVEdge *addEdge(const QString &source, const QString &target,
        const QString &label);

    QGVSubGraph* addSubGraph(const QString& name, bool cluster = true);
    void removeNode(const QString &label);
//    void removeEdge(const QString& source, const QString& target);
//    void removeEdge(const QPair<QString, QString>& key);
    void removeEdge(const QString &label);

    bool hasNode(const QString &name);
    QGVNode *getNode(const QString &name);

    bool hasEdge(const QString &_label);
    QGVEdge *getEdge(const QString &_label);

    void setRootNode(QGVNode *node);

    int nodeCount() const;
    int edgeCount() const;

    void loadLayout(const QString &text);
    void applyLayout();
    void clearLayout();
    void clear();


  signals:
    void nodeContextMenu(QGVNode* node);
    void nodeDoubleClick(QGVNode* node);

    void edgeContextMenu(QGVEdge* edge);
    void edgeDoubleClick(QGVEdge* edge);

    void subGraphContextMenu(QGVSubGraph* graph);
    void subGraphDoubleClick(QGVSubGraph* graph);

    void graphContextMenuEvent();

  public slots:

  protected:
    virtual void contextMenuEvent(
        QGraphicsSceneContextMenuEvent * contextMenuEvent);
    virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * mouseEvent);
    virtual void drawBackground(QPainter * painter, const QRectF & rect);
  private:
    friend class QGVNode;
    friend class QGVEdge;
    friend class QGVSubGraph;

    QGVGvcPrivate *_context;
    QGVGraphPrivate *_graph;

    QMap<QString, QGVNode *> _nodes;
//    QMap<QPair<QString, QString>, QGVEdge*> _edges;
    QMap<QString, QGVEdge *> _edges;
    QMap<QString, QGVSubGraph *> _subGraphs;

    bool init;
};

#endif
