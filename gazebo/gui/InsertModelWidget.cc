/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "sdf/sdf.hh"
#include "common/SystemPaths.hh"
#include "common/Console.hh"

#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"
#include "gui/GuiEvents.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/InsertModelWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
InsertModelWidget::InsertModelWidget(QWidget *_parent)
: QWidget(_parent)
{
  this->setObjectName("insertModel");

  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->fileTreeWidget = new QTreeWidget();
  this->fileTreeWidget->setColumnCount(1);
  this->fileTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->fileTreeWidget->header()->hide();
  connect(this->fileTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnModelSelection(QTreeWidgetItem *, int)));

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->addWidget(this->fileTreeWidget, 0);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);


  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  std::list<std::string> gazeboPaths =
    common::SystemPaths::Instance()->GetGazeboPaths();

  // Iterate over all the gazebo paths
  for (std::list<std::string>::iterator iter = gazeboPaths.begin();
      iter != gazeboPaths.end(); ++iter)
  {
    // This is the full model path
    std::string path = (*iter) +
      common::SystemPaths::Instance()->GetModelPathExtension();

    // Create a top-level tree item for the path
    QTreeWidgetItem *topItem =
      new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
          QStringList(QString("%1").arg(QString::fromStdString(path))));
    this->fileTreeWidget->addTopLevelItem(topItem);

    boost::filesystem::path dir(path);
    boost::filesystem::directory_iterator endIter;
    std::list<boost::filesystem::path> resultSet;

    if (boost::filesystem::exists(dir) &&
        boost::filesystem::is_directory(dir))
    {
      // Iterate over all the model in the current gazebo path
      for (boost::filesystem::directory_iterator dirIter(dir);
          dirIter != endIter; ++dirIter)
      {
        if (boost::filesystem::is_regular_file(dirIter->status()))
        {
          // This is for boost::filesystem version 3+
          // std::string modelName = dirIter->path().filename().string();
          std::string modelName = dirIter->path().filename();

          if (modelName.find(".model") != std::string::npos)
          {
            // Add a child item for the model
            QTreeWidgetItem *childItem = new QTreeWidgetItem(topItem,
                QStringList(QString("%1").arg(
                    QString::fromStdString(dirIter->path().filename()))));
            this->fileTreeWidget->addTopLevelItem(childItem);
          }
        }
      }
    }

    // Make all top-level items expanded. Trying to reduce mouse clicks.
    this->fileTreeWidget->expandItem(topItem);
  }
}

/////////////////////////////////////////////////
InsertModelWidget::~InsertModelWidget()
{
}

/////////////////////////////////////////////////
void InsertModelWidget::OnModelSelection(QTreeWidgetItem *_item,
    int /*_column*/)
{
  if (_item)
  {
    std::string path, filename;

    QApplication::setOverrideCursor(Qt::BusyCursor);

    if (_item->parent())
      path = _item->parent()->text(0).toStdString() + "/";

    filename = _item->text(0).toStdString();

    if (filename.find(".model") == std::string::npos)
      return;

    gui::Events::createEntity("model", path + filename);
    this->fileTreeWidget->clearSelection();

    QApplication::setOverrideCursor(Qt::ArrowCursor);
  }
}

/////////////////////////////////////////////////
/*void InsertModelWidget::OnMouseRelease(const common::MouseEvent &_event)
{
  if (!this->modelSDF || _event.dragging ||
      (_event.button != common::MouseEvent::LEFT &&
       _event.button != common::MouseEvent::RIGHT))
  {
    return;
  }

  if (_event.button == common::MouseEvent::LEFT)
  {
    sdf::ElementPtr modelElem = this->modelSDF->root->GetElement("model");
    std::string modelName = modelElem->GetValueString("name");

    // Automatically create a new name if the model exists
    int i = 0;
    while (has_entity_name(modelName))
    {
      modelName = modelElem->GetValueString("name") + "_" +
                  boost::lexical_cast<std::string>(i++);
    }

    // Remove the topic namespace from the model name. This will get re-inserted
    // by the World automatically
    modelName.erase(0, this->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelName);
    modelElem->GetOrCreateElement("origin")->GetAttribute("pose")->Set(
        this->modelVisual->GetWorldPose());

    // Spawn the model in the physics server
    msgs::Factory msg;
    msg.set_sdf(this->modelSDF->ToString());
    this->factoryPub->Publish(msg);
  }

  // Remove the temporary visual from the scene
  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  scene->RemoveVisual(this->modelVisual);
  this->modelVisual.reset();
  this->visuals.clear();

  this->fileTreeWidget->clearSelection();

  this->modelSDF.reset();
}*/
