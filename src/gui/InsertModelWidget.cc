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
#include <QtGui>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"
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

  InsertModelWidget::InsertModelWidget(QWidget *_parent)
: QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->fileTreeWidget = new QTreeWidget();
  this->fileTreeWidget->setColumnCount(1);
  this->fileTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->fileTreeWidget->header()->hide();
  connect(this->fileTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnModelSelection(QTreeWidgetItem *, int)));

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  this->addButton = new QPushButton(tr("Apply"));
  connect(this->addButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  this->cancelButton = new QPushButton(tr("Cancel"));
  connect(this->cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  buttonLayout->addWidget(this->addButton);
  buttonLayout->addWidget(this->cancelButton);

  mainLayout->addWidget(this->fileTreeWidget);
  mainLayout->addLayout(buttonLayout);
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
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->visualPub = this->node->Advertise<msgs::Visual>("~/visual");
  this->selectionPub = this->node->Advertise<msgs::Selection>("~/selection");
}

InsertModelWidget::~InsertModelWidget()
{
}

void InsertModelWidget::OnModelSelection(QTreeWidgetItem *_item,
    int /*_column*/)
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  scene->RemoveVisual(this->modelVisual);
  this->modelVisual.reset();
  this->visuals.clear();

  if (_item)
  {
    std::string path, filename;

    QApplication::setOverrideCursor(Qt::BusyCursor);

    if (_item->parent())
      path = _item->parent()->text(0).toStdString() + "/";

    filename = _item->text(0).toStdString();

    if (filename.find(".model") == std::string::npos)
      return;

    this->modelSDF.reset(new sdf::SDF);
    sdf::initFile("/sdf/gazebo.sdf", this->modelSDF);
    sdf::readFile(path+filename, this->modelSDF);

    // Load the world file
    std::string modelName;
    math::Pose modelPose, linkPose, visualPose;

    sdf::ElementPtr modelElem = this->modelSDF->root->GetElement("model");
    if (modelElem->HasElement("origin"))
      modelPose = modelElem->GetElement("origin")->GetValuePose("pose");

    modelName = this->node->GetTopicNamespace() + "::" +
      modelElem->GetValueString("name");

    this->modelVisual.reset(new rendering::Visual(modelName,
          scene->GetWorldVisual()));
    this->modelVisual->Load();
    this->modelVisual->SetPose(modelPose);

    modelName = this->modelVisual->GetName();
    modelElem->GetAttribute("name")->Set(modelName);

    scene->AddVisual(this->modelVisual);

    sdf::ElementPtr linkElem = modelElem->GetElement("link");
    while (linkElem)
    {
      std::string linkName = linkElem->GetValueString("name");
      if (linkElem->HasElement("origin"))
        linkPose = linkElem->GetElement("origin")->GetValuePose("pose");


      rendering::VisualPtr linkVisual(new rendering::Visual(modelName + "::" +
            linkName, this->modelVisual));
      linkVisual->Load();
      linkVisual->SetPose(linkPose);
      this->visuals.push_back(linkVisual);

      int visualIndex = 0;
      sdf::ElementPtr visualElem;

      if (linkElem->HasElement("visual"))
        visualElem = linkElem->GetElement("visual");

      while (visualElem)
      {
        if (visualElem->HasElement("origin"))
          visualPose = visualElem->GetElement("origin")->GetValuePose("pose");

        std::ostringstream visualName;
        visualName << modelName << "::" << linkName << "::Visual_"
          << visualIndex++;
        rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
              linkVisual));
        visVisual->Load(visualElem);
        this->visuals.push_back(visVisual);


        visualElem = visualElem->GetNextElement();
      }

      linkElem = linkElem->GetNextElement();
    }

    msgs::Selection selectMsg;
    selectMsg.set_name(modelName);
    selectMsg.set_selected(true);
    selectMsg.set_id(gui::get_entity_id(modelName));
    this->selectionPub->Publish(selectMsg);

    QApplication::setOverrideCursor(Qt::ArrowCursor);
  }
}

void InsertModelWidget::OnApply()
{
  msgs::Factory msg;
  msg.set_sdf(this->modelSDF->ToString());

  sdf::ElementPtr modelElem = this->modelSDF->root->GetElement("model");
  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  std::string modelName = modelElem->GetValueString("name");

  // Remove the selection
  msgs::Selection selectMsg;
  selectMsg.set_name(modelName);
  selectMsg.set_selected(false);
  selectMsg.set_id(gui::get_entity_id(modelName));
  this->selectionPub->Publish(selectMsg);

  // Remove the topic namespace from the model name. This will get re-inserted
  // by the World automatically
  modelName.erase(0, this->node->GetTopicNamespace().size()+2);

  // The the SDF model's name
  modelElem->GetAttribute("name")->Set(modelName);
  modelElem->GetOrCreateElement("origin")->GetAttribute("pose")->Set(
      this->modelVisual->GetWorldPose());

  // Remove the temporary visual from the scene
  scene->RemoveVisual(this->modelVisual);
  this->modelVisual.reset();
  this->visuals.clear();

  this->factoryPub->Publish(msg);
  this->fileTreeWidget->clearSelection();
}

void InsertModelWidget::OnCancel()
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  scene->RemoveVisual(this->modelVisual);
  this->modelVisual.reset();
  this->visuals.clear();

  // Get the name of the model
  std::string modelName =
    this->modelSDF->root->GetElement("model")->GetValueString("name");

  // Remove the selection
  msgs::Selection selectMsg;
  selectMsg.set_name(modelName);
  selectMsg.set_selected(false);
  selectMsg.set_id(gui::get_entity_id(modelName));
  this->selectionPub->Publish(selectMsg);
  this->fileTreeWidget->clearSelection();
}


