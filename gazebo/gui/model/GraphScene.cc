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

#include "gazebo/gui/model/GraphScene.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GraphScene::GraphScene(QWidget *_parent)
  : QGVScene("schematic_view_scene", _parent)
{
  this->setObjectName("GraphScene");

  //Configure scene attributes
  this->setGraphAttribute("splines", "ortho");
  this->setGraphAttribute("rankdir", "LR");
  //_scene->setGraphAttribute("concentrate", "true"); //Error !
  this->setGraphAttribute("nodesep", "0.4");
  this->setNodeAttribute("shape", "box");
  this->setNodeAttribute("style", "filled");
  this->setNodeAttribute("fillcolor", "white");
  this->setNodeAttribute("height", "1.2");
  this->setEdgeAttribute("minlen", "3");
}

/////////////////////////////////////////////////
GraphScene::~GraphScene()
{
}

/////////////////////////////////////////////////
QGVNode *GraphScene::AddNode(const std::string &_name)
{
  // Note don't show the ground plane
  if (_name.find("plane") != std::string::npos)
    return NULL;

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
QGVEdge *GraphScene::AddEdge(const std::string &_node1,
    const std::string &_node2)
{
  QGVEdge *edge = this->addEdge(tr(_node1.c_str()), tr(_node2.c_str()));
  return edge;
}

/////////////////////////////////////////////////
void GraphScene::RemoveEdge(const std::string &_node1,
    const std::string &_node2)
{
  this->removeEdge(tr(_node1.c_str()), tr(_node2.c_str()));
}

/////////////////////////////////////////////////
void GraphScene::drawBackground(QPainter * _painter, const QRectF & _rect)
{
  QColor c(250, 250, 250);
  QBrush brush(c, Qt::SolidPattern);
  _painter->setBrush(brush);
  _painter->drawRect(_rect);

  const int gridSize = 25;

  const qreal left = int(_rect.left()) - (int(_rect.left()) % gridSize);
  const qreal top = int(_rect.top()) - (int(_rect.top()) % gridSize);

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
