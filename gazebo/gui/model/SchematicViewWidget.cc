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

#include "gazebo/common/Events.hh"

#include "gazebo/gui/model/GraphScene.hh"
#include "gazebo/gui/model/GraphView.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/SchematicViewWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
SchematicViewWidget::SchematicViewWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("SchematicViewWidget");

  this->scene = new GraphScene(this);
  this->view = new GraphView(_parent);

  /*QColor c(220, 220, 220);
  QBrush brush(c, Qt::SolidPattern);
  view->setBackgroundBrush(brush);*/

/*  this->minimumWidth = 1240*2;
  this->minimumHeight = 1024*2;
  this->scene->setSceneRect(-this->minimumWidth/2, -this->minimumHeight/2,
      this->minimumWidth, this->minimumHeight);*/
  QHBoxLayout *canvasLayout = new QHBoxLayout(this);
  canvasLayout->addWidget(view);
  canvasLayout->setAlignment(Qt::AlignHCenter);

  this->view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  this->view->setScene(this->scene);
  this->view->centerOn(QPointF(0, 0));
//  this->view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  this->view->setDragMode(QGraphicsView::ScrollHandDrag);
  this->view->show();

  canvasLayout->setContentsMargins(0, 0, 0, 0);
  canvasLayout->setSpacing(0);
  this->setLayout(canvasLayout);
}

/////////////////////////////////////////////////
SchematicViewWidget::~SchematicViewWidget()
{
}

/////////////////////////////////////////////////
void SchematicViewWidget::Reset()
{
  this->scene->clearLayout();
  this->edges.clear();
}

/////////////////////////////////////////////////
void SchematicViewWidget::Init()
{
/*  this->connections.push_back(gui::model::Events::ConnectLinkInserted(
      boost::bind(&SchematicViewWidget::AddNode, this, _1)));

  this->connections.push_back(gui::model::Events::ConnectLinkRemoved(
      boost::bind(&SchematicViewWidget::RemoveNode, this, _1)));

  this->connections.push_back(gui::model::Events::ConnectJointInserted(
      boost::bind(&SchematicViewWidget::AddEdge, this, _1, _2, _3, _4)));

  this->connections.push_back(gui::model::Events::ConnectJointRemoved(
      boost::bind(&SchematicViewWidget::RemoveEdge, this, _1, _2)));

  this->connections.push_back(gui::model::Events::ConnectJointNameChanged(
      boost::bind(&SchematicViewWidget::RenameEdge, this, _1, _2)));
  */
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddNode(const std::string &_node)
{
  std::cerr << "Add node " << _node << std::endl;
  if (this->scene->HasNode(_node))
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();
  this->scene->AddNode(_node);
  //Layout scene
  this->scene->applyLayout();
}

/////////////////////////////////////////////////
void SchematicViewWidget::RemoveNode(const std::string &_node)
{
  std::cerr << "remove node " << _node << std::endl;
  if (this->scene->HasNode(_node))
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();
  this->scene->RemoveNode(_node);

  //Layout scene
  this->scene->applyLayout();
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddEdge(const std::string &_id,
    const std::string &/*_name*/, const std::string &_parent,
    const std::string &_child)
{
  this->edges[_id] = std::make_pair(_parent, _child);

  std::cerr << "add edge " << _parent << " " << _child << std::endl;
  // this must be called before making changes to the graph
  this->scene->clearLayout();
  this->scene->AddEdge(_parent, _child);
  //Layout scene
  this->scene->applyLayout();
}

/////////////////////////////////////////////////
void SchematicViewWidget::RemoveEdge(const std::string &_id,
    const std::string &/*_name*/)
{
  auto it = this->edges.find(_id);
  if (it != this->edges.end())
  {
    std::string parentNode = it->second.first;
    std::string childNode = it->second.second;
    // this must be called before making changes to the graph
    this->scene->clearLayout();
    this->scene->RemoveEdge(parentNode, childNode);
    //Layout scene
    this->scene->applyLayout();

    this->edges.erase(it);
    std::cerr << "remove edge " << parentNode << " " << childNode << std::endl;
  }
}

/*/////////////////////////////////////////////////
void SchematicViewWidget::RenameEdge(const std::string &_id,
    const std::string &_name)
{
  auto it = this->edges.find(_id);
  if (it != this->edges.end())
  {
    std::string parentNode = it->second.first;
    std::string childNode = it->second.second;
    // this must be called before making changes to the graph
    this->scene->clearLayout();
    this->scene->RemoveEdge(parentNode, childNode);
    //Layout scene
    this->scene->applyLayout();

    this->edges.erase(it);
    std::cerr << "remove edge " << parentNode << " " << childNode << std::endl;
  }
}*/


/////////////////////////////////////////////////
void SchematicViewWidget::resizeEvent(QResizeEvent *_event)
{
/*  qreal boundingWidth = std::max(this->minimumWidth, _event->size().width());
  boundingWidth = std::max(boundingWidth, this->scene->sceneRect().width());
  qreal boundingHeight = std::max(this->minimumHeight,
      _event->size().height());
  boundingHeight = std::max(boundingHeight, this->scene->sceneRect().height());
  this->scene->setSceneRect(-boundingWidth/2, -boundingHeight/2,
      boundingWidth, boundingHeight);*/
}
