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

#include "gazebo/rendering/Material.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Color.hh"

#include "gazebo/gui/model/JointMaker.hh"
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

  this->minimumWidth = 500;
  this->minimumHeight = 500;

  QHBoxLayout *canvasLayout = new QHBoxLayout(this);
  canvasLayout->addWidget(view);
  canvasLayout->setAlignment(Qt::AlignHCenter);

  this->view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  this->view->setScene(this->scene);
  this->view->centerOn(QPointF(0, 0));
  this->view->setDragMode(QGraphicsView::ScrollHandDrag);
  this->view->show();

  canvasLayout->setContentsMargins(0, 0, 0, 0);
  canvasLayout->setSpacing(0);
  this->setLayout(canvasLayout);
}

/////////////////////////////////////////////////
void SchematicViewWidget::Reset()
{
  this->edges.clear();
  this->scene->clear();
  this->FitInView();
}

/////////////////////////////////////////////////
void SchematicViewWidget::Init()
{
  this->connections.push_back(gui::model::Events::ConnectLinkInserted(
      boost::bind(&SchematicViewWidget::AddNode, this, _1)));

  this->connections.push_back(gui::model::Events::ConnectLinkRemoved(
      boost::bind(&SchematicViewWidget::RemoveNode, this, _1)));

  this->connections.push_back(gui::model::Events::ConnectJointInserted(
      boost::bind(&SchematicViewWidget::AddEdge, this, _1, _2, _3, _4, _5)));

  this->connections.push_back(gui::model::Events::ConnectJointRemoved(
      boost::bind(&SchematicViewWidget::RemoveEdge, this, _1)));

  this->connections.push_back(gui::model::Events::ConnectJointChanged(
      boost::bind(&SchematicViewWidget::UpdateEdge, this, _1, _2, _3, _4, _5)));
}

/////////////////////////////////////////////////
std::string SchematicViewWidget::GetLeafName(const std::string &_scopedName)
{
  if (_scopedName.empty())
    return "";

  std::string leafName = _scopedName;
  size_t idx = _scopedName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = _scopedName.substr(idx+1);
  return leafName;
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddNode(const std::string &_node)
{
  std::string node = this->GetLeafName(_node);

  if (this->scene->HasNode(node))
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();

  this->scene->AddNode(node);
  this->scene->applyLayout();

  this->FitInView();
}

/////////////////////////////////////////////////
unsigned int SchematicViewWidget::GetNodeCount() const
{
  return static_cast<unsigned int>(this->scene->nodeCount());
}

/////////////////////////////////////////////////
void SchematicViewWidget::RemoveNode(const std::string &_node)
{
  std::string node = this->GetLeafName(_node);

  if (!this->scene->HasNode(node))
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();
  this->scene->RemoveNode(node);

  this->scene->applyLayout();
  this->FitInView();
}

/////////////////////////////////////////////////
bool SchematicViewWidget::HasNode(const std::string &_name) const
{
  return this->scene->HasNode(_name);
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddEdge(const std::string &_id,
    const std::string &/*_name*/, const std::string &_type,
    const std::string &_parent, const std::string &_child)
{
  std::string parentNode = this->GetLeafName(_parent);
  std::string childNode = this->GetLeafName(_child);

  this->edges[_id] = std::make_pair(parentNode, childNode);

  // this must be called before making changes to the graph
  this->scene->clearLayout();

  this->scene->AddEdge(_id, parentNode, childNode);

  std::string materialName = JointMaker::GetJointMaterial(_type);

  common::Color edgeColor = common::Color::Black;
  if (!materialName.empty())
  {
    common::Color emptyColor;
    common::Color matAmbient;
    common::Color matDiffuse;
    common::Color matSpecular;
    common::Color matEmissive;
    rendering::Material::GetMaterialAsColor(materialName, matAmbient,
        matDiffuse, matSpecular, matEmissive);
    edgeColor = matDiffuse;
  }

  this->scene->SetEdgeColor(_id, edgeColor);

  this->scene->applyLayout();

  this->FitInView();
}

/////////////////////////////////////////////////
void SchematicViewWidget::RemoveEdge(const std::string &_id)
{
  auto it = this->edges.find(_id);
  if (it != this->edges.end())
  {
    // this must be called before making changes to the graph
    this->scene->clearLayout();

    this->scene->RemoveEdge(_id);
    this->scene->applyLayout();

    this->FitInView();

    this->edges.erase(it);
  }
}

/////////////////////////////////////////////////
void SchematicViewWidget::UpdateEdge(const std::string &_id,
    const std::string &_name, const std::string &_type,
    const std::string &_parent, const std::string &_child)
{
  auto it = this->edges.find(_id);
  if (it != this->edges.end())
  {
    this->scene->RemoveEdge(_id);
    this->edges.erase(it);

    this->AddEdge(_id, _name, _type, _parent, _child);
  }
}

/////////////////////////////////////////////////
unsigned int SchematicViewWidget::GetEdgeCount() const
{
  return static_cast<unsigned int>(this->scene->edgeCount());
}

/////////////////////////////////////////////////
void SchematicViewWidget::resizeEvent(QResizeEvent */*_event*/)
{
  this->FitInView();
}

/////////////////////////////////////////////////
void SchematicViewWidget::FitInView()
{
  QRectF newRect;
  QRectF sceneRect = this->scene->itemsBoundingRect();

  int sceneCenterX = sceneRect.x() + sceneRect.width()*0.5;
  int sceneCenterY = sceneRect.y() + sceneRect.height()*0.5;

  int sceneWidth = std::max(static_cast<int>(sceneRect.width()),
      this->minimumWidth);
  int sceneHeight = std::max(static_cast<int>(sceneRect.height()),
      this->minimumHeight);

  newRect.setX(sceneCenterX - sceneWidth*0.5);
  newRect.setY(sceneCenterY - sceneHeight*0.5);
  newRect.setWidth(sceneWidth);
  newRect.setHeight(sceneHeight);

  this->view->fitInView(newRect, Qt::KeepAspectRatio);
  this->view->centerOn(sceneCenterX, sceneCenterY);
  this->scene->setSceneRect(newRect);
}
