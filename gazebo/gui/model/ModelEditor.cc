/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <boost/bind/bind.hpp>
#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/TopToolbar.hh"
#include "gazebo/gui/model/EditorMaterialSwitcher.hh"
#include "gazebo/gui/model/ModelTreeWidget.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorPrivate.hh"
#include "gazebo/gui/model/ModelEditor.hh"
#include "gazebo/gui/model/ModelEditorTypes.hh"

#ifdef HAVE_GRAPHVIZ
#include "gazebo/gui/model/SchematicViewWidget.hh"
#endif

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditor::ModelEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow), dataPtr(new ModelEditorPrivate)
{
  this->dataPtr->active = false;
  // Create the model editor palette tab
  this->dataPtr->modelPalette = new ModelEditorPalette(_mainWindow);
  // create the model tree tab
  this->dataPtr->modelTree = new ModelTreeWidget(_mainWindow);
  this->dataPtr->modelTree->hide();
  this->Init("modelEditorTab", "Insert", this->dataPtr->modelPalette);
  this->tabWidget->addTab(this->dataPtr->modelTree, tr("Model"));

  GZ_ASSERT(this->tabWidget != NULL, "Editor tab widget is NULL");

  this->dataPtr->schematicViewAct = NULL;
  this->dataPtr->svWidget = NULL;
#ifdef HAVE_GRAPHVIZ
  RenderWidget *renderWidget = _mainWindow->RenderWidget();
  this->dataPtr->svWidget = new SchematicViewWidget(renderWidget);
  this->dataPtr->svWidget->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->dataPtr->svWidget->Init();
  renderWidget->InsertWidget(0, this->dataPtr->svWidget);
  this->dataPtr->svWidget->hide();

  this->dataPtr->schematicViewAct =
      new QAction(tr("Schematic View"), this->mainWindow);
  this->dataPtr->schematicViewAct->setStatusTip(tr("Sch&ematic View"));
  this->dataPtr->schematicViewAct->setShortcut(tr("Ctrl+E"));
  this->dataPtr->schematicViewAct->setCheckable(true);
  connect(this->dataPtr->schematicViewAct, SIGNAL(toggled(bool)), this,
      SLOT(OnSchematicView(bool)));
#endif

  this->dataPtr->newAct = new QAction(tr("&New"), this->mainWindow);
  this->dataPtr->newAct->setStatusTip(tr("New"));
  this->dataPtr->newAct->setShortcut(tr("Ctrl+N"));
  this->dataPtr->newAct->setCheckable(false);
  connect(this->dataPtr->newAct, SIGNAL(triggered()), this, SLOT(New()));

  this->dataPtr->saveAct = new QAction(tr("&Save"), this->mainWindow);
  this->dataPtr->saveAct->setStatusTip(tr("Save"));
  this->dataPtr->saveAct->setShortcut(tr("Ctrl+S"));
  this->dataPtr->saveAct->setCheckable(false);
  connect(this->dataPtr->saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  this->dataPtr->saveAsAct = new QAction(tr("&Save As"), this->mainWindow);
  this->dataPtr->saveAsAct->setStatusTip(tr("Save As"));
  this->dataPtr->saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  this->dataPtr->saveAsAct->setCheckable(false);
  connect(this->dataPtr->saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));

  this->dataPtr->exitAct = new QAction(tr("E&xit Model Editor"),
      this->mainWindow);
  this->dataPtr->exitAct->setStatusTip(tr("Exit Model Editor"));
  this->dataPtr->exitAct->setShortcut(tr("Ctrl+X"));
  this->dataPtr->exitAct->setCheckable(false);
  connect(this->dataPtr->exitAct, SIGNAL(triggered()), this, SLOT(Exit()));

  this->dataPtr->showCollisionsAct = new QAction(tr("Collisions"), this);
  this->dataPtr->showCollisionsAct->setStatusTip(tr("Show Collisions"));
  this->dataPtr->showCollisionsAct->setCheckable(true);
  this->dataPtr->showCollisionsAct->setChecked(true);
  this->connect(this->dataPtr->showCollisionsAct, SIGNAL(toggled(bool)),
      this->dataPtr->modelPalette->ModelCreator(), SLOT(ShowCollisions(bool)));

  this->dataPtr->showVisualsAct = new QAction(tr("Visuals"), this);
  this->dataPtr->showVisualsAct->setStatusTip(tr("Show Visuals"));
  this->dataPtr->showVisualsAct->setCheckable(true);
  this->dataPtr->showVisualsAct->setChecked(true);
  this->connect(this->dataPtr->showVisualsAct, SIGNAL(toggled(bool)),
      this->dataPtr->modelPalette->ModelCreator(), SLOT(ShowVisuals(bool)));

  this->dataPtr->showJointsAct = new QAction(tr("Joints"), this);
  this->dataPtr->showJointsAct->setStatusTip(tr("Show Joints"));
  this->dataPtr->showJointsAct->setCheckable(true);
  this->dataPtr->showJointsAct->setChecked(true);
  connect(this->dataPtr->showJointsAct, SIGNAL(toggled(bool)),
      this->dataPtr->modelPalette->ModelCreator()->JointMaker(),
      SLOT(ShowJoints(bool)));

  this->dataPtr->showLinkFramesAct = new QAction(tr("Link Frames"), this);
  this->dataPtr->showLinkFramesAct->setStatusTip(tr("Show Link Frames"));
  this->dataPtr->showLinkFramesAct->setCheckable(true);
  this->dataPtr->showLinkFramesAct->setChecked(true);
  this->connect(this->dataPtr->showLinkFramesAct, SIGNAL(toggled(bool)),
      this->dataPtr->modelPalette->ModelCreator(), SLOT(ShowLinkFrames(bool)));

  // Clone actions from main window
  this->dataPtr->showToolbarsAct =
      this->mainWindow->CloneAction(g_showToolbarsAct, this);
  this->dataPtr->fullScreenAct =
      this->mainWindow->CloneAction(g_fullScreenAct, this);
  this->dataPtr->cameraOrthoAct =
      this->mainWindow->CloneAction(g_cameraOrthoAct, this);
  this->dataPtr->cameraPerspectiveAct =
      this->mainWindow->CloneAction(g_cameraPerspectiveAct, this);

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::model::Events::ConnectFinishModel(
      boost::bind(&ModelEditor::OnFinish, this)));

  // Add a joint icon to the toolbar
  this->dataPtr->jointAct  = new QAction(QIcon(":/images/draw_link.svg"),
      tr("Joint"), this);
  this->dataPtr->jointAct->setCheckable(true);
  this->dataPtr->jointAct->setObjectName("modelEditorJointAct");

  QToolButton *jointButton = new QToolButton();
  jointButton->setObjectName("jointToolButton");
  jointButton->setCheckable(false);
  jointButton->setFixedWidth(15);
  jointButton->setPopupMode(QToolButton::InstantPopup);
  QMenu *jointMenu = new QMenu(jointButton);
  jointButton->setMenu(jointMenu);
  QAction *revoluteJointAct = new QAction(tr("Revolute"), this);
  QAction *revolute2JointAct = new QAction(tr("Revolute2"), this);
  QAction *prismaticJointAct = new QAction(tr("Prismatic"), this);
  QAction *ballJointAct = new QAction(tr("Ball"), this);
  QAction *universalJointAct = new QAction(tr("Universal"), this);
  QAction *screwJointAct = new QAction(tr("Screw"), this);
  QAction *gearboxJointAct = new QAction(tr("Gearbox"), this);
  QAction *fixedJointAct = new QAction(tr("Fixed"), this);

  revoluteJointAct->setCheckable(true);
  revolute2JointAct->setCheckable(true);
  prismaticJointAct->setCheckable(true);
  ballJointAct->setCheckable(true);
  universalJointAct->setCheckable(true);
  screwJointAct->setCheckable(true);
  gearboxJointAct->setCheckable(true);
  fixedJointAct->setCheckable(true);

  jointMenu->addAction(revoluteJointAct);
  jointMenu->addAction(revolute2JointAct);
  jointMenu->addAction(prismaticJointAct);
  jointMenu->addAction(ballJointAct);
  jointMenu->addAction(universalJointAct);
  jointMenu->addAction(screwJointAct);
  jointMenu->addAction(gearboxJointAct);
  jointMenu->addAction(fixedJointAct);

  QActionGroup *jointActionGroup = new QActionGroup(this);
  jointActionGroup->addAction(revoluteJointAct);
  jointActionGroup->addAction(revolute2JointAct);
  jointActionGroup->addAction(prismaticJointAct);
  jointActionGroup->addAction(ballJointAct);
  jointActionGroup->addAction(universalJointAct);
  jointActionGroup->addAction(screwJointAct);
  jointActionGroup->addAction(gearboxJointAct);
  jointActionGroup->addAction(fixedJointAct);
  jointActionGroup->setExclusive(true);

  TopToolbar *topToolbar = this->mainWindow->RenderWidget()->GetToolbar();

  // Separator
  QAction *jointSeparatorAct =
      topToolbar->InsertSeparator("toolbarSpacerAction");
  jointSeparatorAct->setObjectName(
      "modelEditorJointSeparatorAct");

  // Joint create action
  topToolbar->InsertAction("toolbarSpacerAction", this->dataPtr->jointAct);

  // Joint type dropdown
  QAction *jointTypeAct = topToolbar->InsertWidget("toolbarSpacerAction",
      jointButton);
  jointTypeAct->setObjectName("modelEditorJointTypeAct");

  this->dataPtr->jointAct->setVisible(false);
  jointSeparatorAct->setVisible(false);
  jointTypeAct->setVisible(false);

  this->dataPtr->signalMapper = new QSignalMapper(this);
  connect(this->dataPtr->signalMapper, SIGNAL(mapped(const QString)),
      this, SLOT(OnAddJoint(const QString)));

  connect(revoluteJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(revoluteJointAct,
      revoluteJointAct->text().toLower());
  connect(revolute2JointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(revolute2JointAct,
      revolute2JointAct->text().toLower());
  connect(prismaticJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(prismaticJointAct,
      prismaticJointAct->text().toLower());
  connect(ballJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(ballJointAct,
      ballJointAct->text().toLower());
  connect(universalJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(universalJointAct,
      universalJointAct->text().toLower());
  connect(screwJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(screwJointAct,
      screwJointAct->text().toLower());
  connect(gearboxJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(gearboxJointAct,
      gearboxJointAct->text().toLower());
  connect(fixedJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(fixedJointAct,
      fixedJointAct->text().toLower());

  // set default joint type.
  revoluteJointAct->setChecked(true);
  this->dataPtr->selectedJointType =
      revoluteJointAct->text().toLower().toStdString();
  connect(this->dataPtr->jointAct, SIGNAL(triggered()), this,
      SLOT(OnAddSelectedJoint()));

  connect(this->dataPtr->modelPalette->ModelCreator()->JointMaker(),
      SIGNAL(JointAdded()), this, SLOT(OnJointAdded()));

  using namespace boost::placeholders;
  this->dataPtr->connections.push_back(
      gui::Events::ConnectCreateEntity(
        boost::bind(&ModelEditor::OnCreateEntity, this, _1, _2)));

  this->dataPtr->menuBar = NULL;
  this->dataPtr->insertModel = NULL;
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor()
{
}

////////////////////////////////////////////////
void ModelEditor::AddItemToPalette(QWidget *_item,
    const std::string &_category)
{
  if (!_item)
  {
    gzerr << "Item is NULL" << std::endl;
    return;
  }

  this->dataPtr->modelPalette->AddItem(_item, _category);
}

////////////////////////////////////////////////
void ModelEditor::Save()
{
  gui::model::Events::saveModelEditor();
}

////////////////////////////////////////////////
void ModelEditor::SaveAs()
{
  gui::model::Events::saveAsModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::New()
{
  gui::model::Events::newModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::Exit()
{
  gui::model::Events::exitModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::OnSchematicView(bool _show)
{
  if (!this->dataPtr->svWidget)
    return;

  if (_show)
  {
#ifdef HAVE_GRAPHVIZ
    this->dataPtr->svWidget->show();
  }
  else
  {
    this->dataPtr->svWidget->hide();
#endif
  }
}

/////////////////////////////////////////////////
void ModelEditor::CreateMenus()
{
  if (this->dataPtr->menuBar)
    return;

  this->dataPtr->menuBar = new QMenuBar;
  this->dataPtr->menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  QMenu *fileMenu = this->dataPtr->menuBar->addMenu(tr("&File"));
  fileMenu->addAction(this->dataPtr->newAct);
  fileMenu->addAction(this->dataPtr->saveAct);
  fileMenu->addAction(this->dataPtr->saveAsAct);
  fileMenu->addAction(this->dataPtr->exitAct);

  QMenu *cameraMenu = this->dataPtr->menuBar->addMenu(tr("&Camera"));
  cameraMenu->addAction(this->dataPtr->cameraOrthoAct);
  cameraMenu->addAction(this->dataPtr->cameraPerspectiveAct);

  QMenu *viewMenu = this->dataPtr->menuBar->addMenu(tr("&View"));
  viewMenu->addAction(this->dataPtr->showCollisionsAct);
  viewMenu->addAction(this->dataPtr->showVisualsAct);
  viewMenu->addAction(this->dataPtr->showJointsAct);
  viewMenu->addAction(this->dataPtr->showLinkFramesAct);

  QMenu *windowMenu = this->dataPtr->menuBar->addMenu(tr("&Window"));
  if (this->dataPtr->schematicViewAct)
  {
    windowMenu->addAction(this->dataPtr->schematicViewAct);
    windowMenu->addSeparator();
  }
  windowMenu->addAction(this->dataPtr->showToolbarsAct);
  windowMenu->addAction(this->dataPtr->fullScreenAct);

  // OSX:
  // There is a problem on osx with the qt5 menubar being out of focus when
  // the application is launched from a terminal, so prevent using a native
  // menubar for now.
  //
  // Ubuntu Xenial + Unity:
  // The native menubar is not registering shortcuts (issue #2134)
  this->dataPtr->menuBar->setNativeMenuBar(false);
}

/////////////////////////////////////////////////
void ModelEditor::OnAddSelectedJoint()
{
  g_arrowAct->trigger();
  this->OnAddJoint(tr(this->dataPtr->selectedJointType.c_str()));
}

/////////////////////////////////////////////////
void ModelEditor::OnAddJoint(const QString &_type)
{
  std::string type = _type.toStdString();
  this->dataPtr->modelPalette->CreateJoint(type);
  this->dataPtr->selectedJointType = type;
  this->dataPtr->jointAct->setChecked(true);
  gui::Events::manipMode("joint");
}

/////////////////////////////////////////////////
void ModelEditor::OnJointAdded()
{
  if (this->dataPtr && this->dataPtr->jointAct &&
      this->dataPtr->jointAct->isChecked())
  {
    this->dataPtr->jointAct->setChecked(false);
    g_arrowAct->trigger();
  }
}

/////////////////////////////////////////////////
void ModelEditor::OnEdit(bool /*_checked*/)
{
  if (!this->dataPtr->active)
  {
    this->CreateMenus();
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("modelEditorTab");
    this->mainWindow->ShowMenuBar(this->dataPtr->menuBar);
    if (!g_showToolbarsAct->isChecked())
      g_showToolbarsAct->trigger();
    this->mainWindow->RenderWidget()->ShowTimePanel(false);
  }
  else
  {
    this->mainWindow->ShowLeftColumnWidget();
    this->mainWindow->ShowMenuBar();
    this->mainWindow->RenderWidget()->ShowTimePanel(true);
  }

#ifdef HAVE_GRAPHVIZ
  if (this->dataPtr->svWidget && this->dataPtr->schematicViewAct)
  {
    this->dataPtr->svWidget->setVisible(
        !this->dataPtr->active && this->dataPtr->schematicViewAct->isChecked());
    if (!this->dataPtr->active)
      this->dataPtr->svWidget->Reset();
  }
#endif

  this->dataPtr->active = !this->dataPtr->active;
  this->ToggleMaterialScheme();
  this->ToggleToolbar();
  this->ToggleInsertWidget();
  // g_editModelAct->setChecked(this->dataPtr->active);
}

/////////////////////////////////////////////////
void ModelEditor::ToggleInsertWidget()
{
  QTabWidget *mainTab = this->mainWindow->findChild<QTabWidget *>("mainTab");
  if (!mainTab)
    return;

  if (!this->dataPtr->active)
  {
    if (this->dataPtr->insertModel)
      this->dataPtr->modelPalette->RemoveWidget(this->dataPtr->insertModel);
    this->dataPtr->modelTree->hide();
    mainTab->setCurrentIndex(0);
    return;
  }

  for (int i = 0; i < mainTab->count(); ++i)
  {
    if (mainTab->tabText(i) == tr("Insert"))
    {
      auto insertModel = mainTab->widget(i);

      // Remove from main tab before inserting in new layout
      mainTab->removeTab(i);

      // Add title
      auto databaseLabel = new QLabel(tr(
          "<font size=4 color='white'>Model Database</font>"));

      // Insert in new layout with title
      auto insertLayout = new QVBoxLayout();
      insertLayout->addWidget(databaseLabel);
      insertLayout->addWidget(insertModel);

      auto insertWidget = new QWidget();
      insertWidget->setLayout(insertLayout);

      this->dataPtr->insertModel = insertWidget;

      // Insert in palette
      this->dataPtr->modelPalette->InsertWidget(1, insertWidget);
      this->dataPtr->modelPalette->show();
      insertModel->show();
      this->tabWidget->setCurrentIndex(0);

      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelEditor::OnFinish()
{
//  this->OnEdit(g_editModelAct->isChecked());
  g_editModelAct->trigger();
}

/////////////////////////////////////////////////
void ModelEditor::ToggleMaterialScheme()
{
  if (this->dataPtr->active)
  {
    if (!this->dataPtr->materialSwitcher)
    {
      rendering::CameraPtr camera =
          boost::dynamic_pointer_cast<rendering::Camera>(
          gui::get_active_camera());
      if (camera)
      {
        this->dataPtr->materialSwitcher.reset(
            new EditorMaterialSwitcher(camera));
      }
      else
      {
        gzerr << "User camera is NULL. "
            << "Non-editable models will keep their original material"
            << std::endl;
      }
    }

    this->dataPtr->materialSwitcher->SetMaterialScheme("ModelEditor");
  }
  else
    this->dataPtr->materialSwitcher->SetMaterialScheme("");
}

/////////////////////////////////////////////////
void ModelEditor::ToggleToolbar()
{
  if (this->dataPtr->active)
    gui::Events::windowMode("ModelEditor");
  else
    gui::Events::windowMode("Simulation");
}

/////////////////////////////////////////////////
void ModelEditor::OnCreateEntity(const std::string &_type,
                                 const std::string &_data)
{
  if (!this->dataPtr->active)
    return;

  if (_type == "model" && !_data.empty())
  {
    sdf::SDFPtr modelSDF(new sdf::SDF);
    sdf::initFile("root.sdf", modelSDF);

    if (!sdf::readFile(_data, modelSDF))
    {
      gzerr << "Unable to load file[" << _data << "]\n";
      return;
    }

    if (modelSDF->Root()->HasElement("model"))
    {
      this->AddEntity(modelSDF->Root()->GetElement("model"));
    }
    else
    {
      gzerr << "No model in SDF\n";
      return;
    }
  }
}

////////////////////////////////////////////////
void ModelEditor::AddEntity(sdf::ElementPtr _sdf)
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();
  this->dataPtr->modelPalette->ModelCreator()->AddEntity(_sdf);
}
