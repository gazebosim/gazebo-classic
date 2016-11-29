/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/qgv/QGVNode.h"
#include "gazebo/gui/qgv/QGVEdge.h"

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
       boost::bind(&SchematicViewWidget::OnDeselectAll, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedEntity(
       boost::bind(&SchematicViewWidget::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedJoint(
       boost::bind(&SchematicViewWidget::OnSetSelectedJoint, this, _1, _2)));
}

/////////////////////////////////////////////////
std::string SchematicViewWidget::UnscopedName(
    const std::string &_scopedName) const
{
  if (_scopedName.empty())
    return "";

  std::string unscopedName = _scopedName;
  size_t idx = _scopedName.find("::");
  if (idx != std::string::npos)
    unscopedName = _scopedName.substr(idx+2);

  return unscopedName;
}

/////////////////////////////////////////////////
std::string SchematicViewWidget::TopLevelName(
    const std::string &_scopedName) const
{
  if (_scopedName.empty())
    return "";

  auto unscopedPos = _scopedName.find("::");

  std::string topLevelName = _scopedName;
  auto secondScopePos = topLevelName.find("::", unscopedPos + 2);
  if (secondScopePos != std::string::npos)
    topLevelName = topLevelName.substr(0, secondScopePos);

  return topLevelName;
}

/////////////////////////////////////////////////
void SchematicViewWidget::AddNode(const std::string &_node)
{
  std::string name = this->UnscopedName(_node);

  if (name.empty() || this->scene->HasNode(name))
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
    std::string node = this->UnscopedName(_node);

    if (node.empty() || !this->scene->HasNode(node))
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
  std::string parentNode = this->UnscopedName(_parent);
  std::string childNode = this->UnscopedName(_child);

  if (parentNode.empty() || childNode.empty())
    return;

  // this must be called before making changes to the graph
  this->scene->clearLayout();

  QGVEdge *edge = this->scene->AddEdge(_id, parentNode, childNode);
  if (!edge)
    return;

  edge->setData(0, tr(_id.c_str()));
  edge->setData(1, tr("Joint"));

  this->edges[_id] = edge;

  std::string materialName = JointMaker::JointMaterial(_type);

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
  else if (this->scene->HasNode(this->UnscopedName(itemId)))
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
void SchematicViewWidget::OnSetSelectedEntity(const std::string &_name,
    bool _selected)
{
  this->scene->blockSignals(true);

  // Select all nodes with the same top level name, so we select all links of a
  // nested model.
  for (auto &node : this->nodes)
  {
    auto topLevelName = this->TopLevelName(node.first);
    if (topLevelName == _name)
    {
      node.second->setSelected(_selected);

      if (!this->selectedItems.contains(node.second))
      {
        this->selectedItems.push_back(node.second);
      }
    }
  }

  this->scene->blockSignals(false);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSetSelectedJoint(const std::string &_id,
    bool _selected)
{
  this->scene->blockSignals(true);

  auto it = this->edges.find(_id);
  if (it != this->edges.end())
  {
    it->second->setSelected(_selected);

    if (!this->selectedItems.contains(it->second))
    {
      this->selectedItems.push_back(it->second);
    }
  }

  this->scene->blockSignals(false);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnDeselectAll(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  this->scene->blockSignals(true);

  // deselect all
  for (auto &node : this->nodes)
    node.second->setSelected(false);

  for (auto &edge : this->edges)
    edge.second->setSelected(false);

  this->selectedItems.clear();

  this->scene->blockSignals(false);
}

/////////////////////////////////////////////////
void SchematicViewWidget::OnSelectionChanged()
{
  QList<QGraphicsItem *> currentlySelected = this->scene->selectedItems();

  // Check if the selection change was deselection
  std::string lastTopLevel("");
  std::string lastType("");
  bool lastIsSelected = true;
  if (this->view->lastClickedItem)
  {
    lastType = this->view->lastClickedItem->data(1).toString().toStdString();

    auto id = this->view->lastClickedItem->data(0).toString().toStdString();
    lastTopLevel = this->TopLevelName(id);

    lastIsSelected = this->view->lastClickedItem->isSelected();
  }

  // Create list of top level names so we can also select siblings
  QList<std::string> currentTopLevel;
  for (auto &item : currentlySelected)
  {
    auto id = item->data(0).toString().toStdString();
    auto topLevel = this->TopLevelName(id);

    auto type = item->data(1).toString().toStdString();

    // If it is a deselection, skip sibling links
    if (!lastIsSelected && lastType == "Link" && type == "Link" &&
        lastTopLevel == topLevel)
    {
      continue;
    }

    if (!currentTopLevel.contains(topLevel))
      currentTopLevel.push_back(topLevel);
  }

  // Update all items
  for (auto &item : this->scene->items())
  {
    auto id = item->data(0).toString().toStdString();
    auto type = item->data(1).toString().toStdString();

    if (type == "Link")
    {
      bool selected;

      // Select if top level matches
      if (lastType == "Link")
      {
        auto topLevelId = this->TopLevelName(id);
        selected = currentTopLevel.contains(topLevelId);
      }
      else
        selected = currentlySelected.contains(item);

      gui::model::Events::setSelectedEntity(id, selected);
    }
    else if (type == "Joint")
    {
      // Select if whole item matches
      bool selected = currentlySelected.contains(item);
      gui::model::Events::setSelectedJoint(id, selected);
    }
    else
      gzwarn << "Unknown type [" << type << "]" << std::endl;
  }

  this->selectedItems = currentlySelected;
}
