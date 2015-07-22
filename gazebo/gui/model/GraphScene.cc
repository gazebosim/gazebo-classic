/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/gui/model/qgv/QGVNode.h"
#include "gazebo/gui/model/qgv/QGVEdge.h"

#include "gazebo/gui/model/GraphScene.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GraphScene::GraphScene(QWidget *_parent)
  : QGVScene("schematic_view_scene", _parent)
{
  this->setObjectName("GraphScene");

  // Configure scene attributes
  this->setGraphAttribute("splines", "ortho");
  this->setGraphAttribute("rankdir", "LR");
  this->setGraphAttribute("nodesep", "0.4");
  this->setNodeAttribute("margin", "0.3");
  this->setNodeAttribute("shape", "box");
  this->setNodeAttribute("style", "filled");
  this->setNodeAttribute("fillcolor", "white");
  this->setNodeAttribute("height", "1.0");
  this->setEdgeAttribute("minlen", "3");
}

/////////////////////////////////////////////////
QGVNode *GraphScene::AddNode(const std::string &_name)
{
  QGVNode *node = this->addNode(tr(_name.c_str()));
  return node;
}

/////////////////////////////////////////////////
void GraphScene::RemoveNode(const std::string &_name)
{
  this->removeNode(tr(_name.c_str()));
}

/////////////////////////////////////////////////
bool GraphScene::HasNode(const std::string &_name)
{
  return this->hasNode(tr(_name.c_str()));
}

/////////////////////////////////////////////////
QGVNode *GraphScene::GetNode(const std::string &_name)
{
  return this->getNode(tr(_name.c_str()));
}

/////////////////////////////////////////////////
QGVEdge *GraphScene::AddEdge(const std::string &_id,
    const std::string &_node1, const std::string &_node2)
{
  QGVEdge *edge = this->addEdge(tr(_node1.c_str()), tr(_node2.c_str()),
      tr(_id.c_str()));
  return edge;
}

/////////////////////////////////////////////////
void GraphScene::RemoveEdge(const std::string &_id)
{
  this->removeEdge(tr(_id.c_str()));
}

/////////////////////////////////////////////////
void GraphScene::SetEdgeColor(const std::string &_id,
    const common::Color &_color)
{
  QGVEdge *edge = this->getEdge(tr(_id.c_str()));
  if (edge)
  {
    edge->setColor(QColor(_color.r*255, _color.g*255, _color.b*255,
        _color.a*255));
  }
}

/////////////////////////////////////////////////
void GraphScene::drawBackground(QPainter *_painter, const QRectF & _rect)
{
  QColor c(250, 250, 250);
  QBrush brush(c, Qt::SolidPattern);
  _painter->setBrush(brush);
  _painter->drawRect(_rect);

  const int gridSize = 25;

  const qreal left = static_cast<int>(_rect.left()) -
      (static_cast<int>(_rect.left()) % gridSize);
  const qreal top = static_cast<int>(_rect.top()) -
      (static_cast<int>(_rect.top()) % gridSize);

  QVarLengthArray<QLineF, 100> lines;

  for (qreal x = left; x < _rect.right(); x += gridSize)
      lines.append(QLineF(x, _rect.top(), x, _rect.bottom()));
  for (qreal y = top; y < _rect.bottom(); y += gridSize)
      lines.append(QLineF(_rect.left(), y, _rect.right(), y));

  _painter->setRenderHint(QPainter::Antialiasing, false);

  _painter->setPen(QColor(200, 200, 200, 125));
  _painter->drawLines(lines.data(), lines.size());
  _painter->setPen(Qt::black);
}
