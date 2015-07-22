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

#include "gazebo/gui/model/qgv/QGVNode.h"
#include "gazebo/gui/model/qgv/QGVEdge.h"

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

  connect(this->view, SIGNAL(customContextMenuRequested(const QString &)),
      this, SLOT(OnCustomContextMenu(const QString &)));

  connect(this->view, SIGNAL(itemDoubleClicked(const QString &)),
      this, SLOT(OnItemDoubleClicked(const QString &)));

  connect(this->scene, SIGNAL(selectionChanged()),
      this, SLOT(OnSelectionChanged()));
}

/////////////////////////////////////////////////
void SchematicViewWidget::Reset()
{
  this->nodes.clear();
  this->edges.clear();
  this->scene->clear();
  this->selectedItems.clear();
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
      
  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&SchematicViewWidget::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedLink(
       boost::bind(&SchematicViewWidget::OnSetSelectedLink, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedJoint(
       boost::bind(&SchematicViewWidget::OnSetSelectedJoint, this, _1, _2)));
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
  std::string name = this->GetLeafName(_node);

  if (this->scene->HasNode(name))
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();

  QGVNode *node = this->scene->AddNode(name);
  node->setData(0, tr(_node.c_str()));
  node->setData(1, tr("Link"));
  this->nodes[_node] = node;

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
  auto it = this->nodes.find(_node);
  if (it != this->nodes.end())
  {
    std::string node = this->GetLeafName(_node);

    if (!this->scene->HasNode(node))
      return;

    // this must be called before making changes to the graph
    this->scene->clearLayout();
    this->scene->RemoveNode(node);

    this->scene->applyLayout();
    this->FitInView();

    this->nodes.erase(it);
  }
}

/////////////////////////////////////////////////
bool SchematicViewWidget::HasNode(const std::string &_name) const
{
  return this->nodes.find(_name) != this->nodes.end();
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddEdge(const std::string &_id,
    const std::string &/*_name*/, const std::string &_type,
    const std::string &_parent, const std::string &_child)
{
  std::string parentNode = this->GetLeafName(_parent);
  std::string childNode = this->GetLeafName(_child);

  // this must be called before making changes to the graph
  this->scene->clearLayout();

  QGVEdge *edge = this->scene->AddEdge(_id, parentNode, childNode);
  edge->setData(0, tr(_id.c_str()));
  edge->setData(1, tr("Joint"));
  this->edges[_id] = edge;

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
bool SchematicViewWidget::HasEdge(const std::string &_id) const
{
  return this->edges.find(_id) != this->edges.end();
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

/////////////////////////////////////////////////
void SchematicViewWidget::OnCustomContextMenu(const QString &_id)
{
  std::string itemId = _id.toStdString();
  if (this->edges.find(itemId) != this->edges.end())
    gui::model::Events::showJointContextMenu(itemId);
  else if (this->scene->HasNode(this->GetLeafName(itemId)))
    gui::model::Events::showLinkContextMenu(itemId);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnItemDoubleClicked(const QString &_id)
{
  std::string itemId = _id.toStdString();
  if (this->HasEdge(itemId))
    gui::model::Events::openJointInspector(itemId);
  else if (this->HasNode(itemId))
    gui::model::Events::openLinkInspector(itemId);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSetSelectedLink(const std::string &_name,
    bool _selected)
{
  auto it = this->nodes.find(_name);
  if (it != this->nodes.end())
    it->second->setSelected(_selected);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSetSelectedJoint(const std::string &_id,
    bool _selected)
{
  auto it = this->edges.find(_id);
  if (it != this->edges.end())
    it->second->setSelected(_selected);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  // deselect all
  for (auto &node : this->nodes)
    node.second->setSelected(false);

  for (auto &edge : this->edges)
    edge.second->setSelected(false);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSelectionChanged()
{
  QList<QGraphicsItem *> items = this->scene->selectedItems();

  // update and signal new selection
  for (auto const item : items)
  {
    int idx = this->selectedItems.indexOf(item);
    if (idx >= 0)
    {
      this->selectedItems.removeAt(idx);
      continue;
    }
    std::string id = item->data(0).toString().toStdString();
    std::string type = item->data(1).toString().toStdString();

    if (type == "Link")
      gui::model::Events::setSelectedLink(id, true);
    else if (type == "Joint")
      gui::model::Events::setSelectedJoint(id, true);
  }

  // deselect
  for (auto const item : this->selectedItems)
  {
    if (item)
    {
      std::string id = item->data(0).toString().toStdString();
      std::string type = item->data(1).toString().toStdString();

      if (type == "Link")
        gui::model::Events::setSelectedLink(id, false);
      else if (type == "Joint")
        gui::model::Events::setSelectedJoint(id, false);
    }
  }

  this->selectedItems = items;
}
