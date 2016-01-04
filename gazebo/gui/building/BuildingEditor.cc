/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <boost/bind.hpp>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/BuildingEditorPalette.hh"
#include "gazebo/gui/building/BuildingEditor.hh"
#include "gazebo/gui/building/BuildingEditorPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditor::BuildingEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow), dataPtr(new BuildingEditorPrivate)
{
  // Tips
  this->dataPtr->tipsLabel = new QLabel(tr(
      "<font size=4 color='white'><b>?</b></font>"));
  this->dataPtr->tipsLabel->setToolTip(tr("<font size=3><p><b> Tips: </b></b>"
      "<p>Double-click an object to open an Inspector with configuration "
      "options.</p>"
      "<p>Currently, windows & doors are simple holes in the wall.</p>"
      "<p>Because Gazebo only supports simple primitive shapes, all floors "
      "will be rectangular.</p>"));
  this->dataPtr->tipsLabel->installEventFilter(this);

  // Create the building editor tab
  this->dataPtr->buildingPalette = new BuildingEditorPalette;
  this->Init("buildingEditorTab", "Building Editor",
      this->dataPtr->buildingPalette, this->dataPtr->tipsLabel);

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

  this->dataPtr->saveAsAct = new QAction(tr("&Save As"),
      this->mainWindow);
  this->dataPtr->saveAsAct->setStatusTip(tr("Save As"));
  this->dataPtr->saveAsAct->setShortcut(tr("Ctrl+SHIFT+S"));
  this->dataPtr->saveAsAct->setCheckable(false);
  connect(this->dataPtr->saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));

  this->dataPtr->exitAct = new QAction(tr("E&xit Building Editor"),
      this->mainWindow);
  this->dataPtr->exitAct->setStatusTip(tr("Exit Building Editor"));
  this->dataPtr->exitAct->setShortcut(tr("Ctrl+X"));
  this->dataPtr->exitAct->setCheckable(false);
  connect(this->dataPtr->exitAct, SIGNAL(triggered()), this, SLOT(Exit()));

  connect(g_editBuildingAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::editor::Events::ConnectFinishBuildingModel(
      boost::bind(&BuildingEditor::OnFinish, this)));

  this->dataPtr->buildingEditorWidget = new BuildingEditorWidget(
      this->mainWindow->GetRenderWidget());
  this->dataPtr->buildingEditorWidget->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->dataPtr->buildingEditorWidget->hide();

  this->mainWindow->GetRenderWidget()->InsertWidget(0,
      this->dataPtr->buildingEditorWidget);

  this->dataPtr->menuBar = NULL;
}

/////////////////////////////////////////////////
BuildingEditor::~BuildingEditor()
{
}

////////////////////////////////////////////////
void BuildingEditor::Save()
{
  gui::editor::Events::saveBuildingEditor();
}

////////////////////////////////////////////////
void BuildingEditor::SaveAs()
{
  gui::editor::Events::saveAsBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditor::New()
{
  gui::editor::Events::newBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditor::Exit()
{
  gui::editor::Events::exitBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditor::OnFinish()
{
  g_editBuildingAct->setChecked(!g_editBuildingAct->isChecked());
  this->OnEdit(false);
}

/////////////////////////////////////////////////
void BuildingEditor::CreateMenus()
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
}

/////////////////////////////////////////////////
void BuildingEditor::OnEdit(bool _checked)
{
  if (_checked)
  {
    this->CreateMenus();
    this->dataPtr->mainWindowPaused = this->mainWindow->IsPaused();
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("buildingEditorTab");
    this->mainWindow->ShowMenuBar(this->dataPtr->menuBar);
    this->dataPtr->buildingEditorWidget->show();
    this->mainWindow->GetRenderWidget()->DisplayOverlayMsg(
        "Building is View Only");
    this->mainWindow->GetRenderWidget()->ShowTimePanel(false);
    this->mainWindow->GetRenderWidget()->ShowToolbar(false);
  }
  else
  {
    this->dataPtr->buildingPalette->CustomColorDialog()->reject();
    this->mainWindow->ShowLeftColumnWidget();
    this->dataPtr->buildingEditorWidget->hide();
    this->mainWindow->GetRenderWidget()->DisplayOverlayMsg("");
    this->mainWindow->GetRenderWidget()->ShowTimePanel(true);
    this->mainWindow->GetRenderWidget()->ShowToolbar(true);
    this->mainWindow->ShowMenuBar();
    if (!this->dataPtr->mainWindowPaused)
      this->mainWindow->Play();
  }
  gui::editor::Events::toggleEditMode(_checked);
}

/////////////////////////////////////////////////
bool BuildingEditor::eventFilter(QObject *_obj, QEvent *_event)
{
  QLabel *label = qobject_cast<QLabel *>(_obj);
  if (label && label == this->dataPtr->tipsLabel &&
      _event->type() == QEvent::MouseButtonRelease)
  {
    QToolTip::showText(this->dataPtr->tipsLabel->mapToGlobal(QPoint()),
        this->dataPtr->tipsLabel->toolTip());
    return true;
  }
  return false;
}
