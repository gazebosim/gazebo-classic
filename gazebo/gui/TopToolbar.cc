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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Console.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/TopToolbarPrivate.hh"
#include "gazebo/gui/TopToolbar.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TopToolbar::TopToolbar(QWidget *_parent)
  : QFrame(_parent), dataPtr(new TopToolbarPrivate)
{
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  this->setObjectName("topToolbar");

  this->dataPtr->toolbar = new QToolBar;
  this->dataPtr->toolbar->setObjectName("topToolbarToolbar");

  // Manipulation modes
  QActionGroup *actionGroup = new QActionGroup(this);
  if (g_arrowAct)
  {
    actionGroup->addAction(g_arrowAct);
    this->dataPtr->toolbar->addAction(g_arrowAct);
  }
  if (g_translateAct)
  {
    actionGroup->addAction(g_translateAct);
    this->dataPtr->toolbar->addAction(g_translateAct);
  }
  if (g_rotateAct)
  {
    actionGroup->addAction(g_rotateAct);
    this->dataPtr->toolbar->addAction(g_rotateAct);
  }
  if (g_scaleAct)
  {
    actionGroup->addAction(g_scaleAct);
    this->dataPtr->toolbar->addAction(g_scaleAct);
  }

  this->dataPtr->toolbar->addSeparator();

  // Undo & Redo
  if (g_undoAct && g_redoAct && g_redoHistoryAct && g_undoHistoryAct)
  {
    this->dataPtr->toolbar->addAction(g_undoAct);
    this->dataPtr->toolbar->addAction(g_undoHistoryAct);
    this->dataPtr->toolbar->addAction(g_redoAct);
    this->dataPtr->toolbar->addAction(g_redoHistoryAct);

    this->dataPtr->toolbar->addSeparator();
  }

  // Insert simple shapes
  if (g_boxCreateAct)
    this->dataPtr->toolbar->addAction(g_boxCreateAct);
  if (g_sphereCreateAct)
    this->dataPtr->toolbar->addAction(g_sphereCreateAct);
  if (g_cylinderCreateAct)
    this->dataPtr->toolbar->addAction(g_cylinderCreateAct);
  this->dataPtr->toolbar->addSeparator();

  // Insert lights
  if (g_pointLghtCreateAct)
    this->dataPtr->toolbar->addAction(g_pointLghtCreateAct);
  if (g_spotLghtCreateAct)
    this->dataPtr->toolbar->addAction(g_spotLghtCreateAct);
  if (g_dirLghtCreateAct)
    this->dataPtr->toolbar->addAction(g_dirLghtCreateAct);
  this->dataPtr->toolbar->addSeparator();

  // Copy & Paste
  if (g_copyAct)
    this->dataPtr->toolbar->addAction(g_copyAct);
  if (g_pasteAct)
    this->dataPtr->toolbar->addAction(g_pasteAct);

  this->dataPtr->toolbar->addSeparator();

  // Align
  if (g_alignAct)
  {
    QToolButton *alignButton = new QToolButton;
    alignButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
    alignButton->setIcon(QIcon(":/images/align.png"));
    alignButton->setToolTip(
        tr("In Selection Mode, hold Ctrl and select 2 objects to align"));
    alignButton->setArrowType(Qt::NoArrow);
    QMenu *alignMenu = new QMenu(alignButton);
    alignMenu->addAction(g_alignAct);
    alignButton->setMenu(alignMenu);
    alignButton->setPopupMode(QToolButton::InstantPopup);
    g_alignButtonAct = this->dataPtr->toolbar->addWidget(alignButton);
    connect(alignButton, SIGNAL(pressed()), g_alignAct, SLOT(trigger()));
  }

  // Snap
  if (g_snapAct)
  {
    actionGroup->addAction(g_snapAct);
    this->dataPtr->toolbar->addAction(g_snapAct);
  }

  this->dataPtr->toolbar->addSeparator();

  // View angle
  if (g_viewAngleAct)
  {
    QToolButton *viewAngleButton = new QToolButton;
    viewAngleButton->setObjectName("viewAngleToolBarButton");
    viewAngleButton->setStyleSheet(
        "#viewAngleToolBarButton{padding-right:10px}");
    viewAngleButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
    viewAngleButton->setIcon(QIcon(":/images/view_angle_front.png"));
    viewAngleButton->setToolTip(tr("Change the view angle"));

    QMenu *viewAngleMenu = new QMenu(viewAngleButton);
    viewAngleMenu->addAction(g_viewAngleAct);

    viewAngleButton->setMenu(viewAngleMenu);
    viewAngleButton->setPopupMode(QToolButton::InstantPopup);
    g_viewAngleButtonAct = this->dataPtr->toolbar->addWidget(viewAngleButton);
  }

  // Empty space to push whatever comes next to the right
  QWidget *spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QAction *spacerAction = this->dataPtr->toolbar->addWidget(spacer);
  spacerAction->setObjectName("toolbarSpacerAction");

  // Screenshot / logging
  if (g_screenshotAct)
    this->dataPtr->toolbar->addAction(g_screenshotAct);
  if (g_dataLoggerAct)
    this->dataPtr->toolbar->addAction(g_dataLoggerAct);

  // Layout
  QHBoxLayout *toolLayout = new QHBoxLayout;
  toolLayout->setContentsMargins(0, 0, 0, 0);
  toolLayout->addSpacing(10);
  toolLayout->addWidget(this->dataPtr->toolbar);
  toolLayout->addSpacing(10);
  this->setLayout(toolLayout);

  // Connections
  this->dataPtr->connections.push_back(
      gui::Events::ConnectWindowMode(
      boost::bind(&TopToolbar::OnWindowMode, this, _1)));
}

/////////////////////////////////////////////////
TopToolbar::~TopToolbar()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void TopToolbar::OnWindowMode(const std::string &_mode)
{
  bool modelEditor = _mode == "ModelEditor";
  bool simulation = _mode == "Simulation";
  bool logPlayback = _mode == "LogPlayback";

  QList<QAction *> acts = this->dataPtr->toolbar->actions();
  for (int i = 0; i < acts.size(); ++i)
  {
    // Simulation / Model Editor / Log Playback
    if (acts[i] == g_screenshotAct ||
        acts[i] == g_viewAngleButtonAct ||
        acts[i]->objectName() == "toolbarSpacerAction")
    {
      acts[i]->setVisible(modelEditor || simulation || logPlayback);
      acts[i]->setEnabled(modelEditor || simulation || logPlayback);
    }
    // Simulation / Model Editor
    else if (acts[i] == g_arrowAct ||
        acts[i] == g_rotateAct ||
        acts[i] == g_translateAct ||
        acts[i] == g_scaleAct ||
        acts[i] == g_copyAct ||
        acts[i] == g_pasteAct ||
        acts[i] == g_alignButtonAct ||
        acts[i] == g_snapAct)
    {
      acts[i]->setVisible(modelEditor || simulation);
      acts[i]->setEnabled(modelEditor || simulation);

      // Change preceding separator as well
      if (i > 0 && acts[i-1]->isSeparator())
      {
        acts[i-1]->setVisible(modelEditor || simulation);
      }
    }
    // Simulation / Log Playback
    else if (acts[i] == g_dataLoggerAct)
    {
      acts[i]->setVisible(simulation || logPlayback);
      acts[i]->setEnabled(simulation || logPlayback);
    }
    // Model Editor only
    else if (acts[i]->objectName().toStdString().find("modelEditor") !=
        std::string::npos)
    {
      acts[i]->setVisible(modelEditor);
      acts[i]->setEnabled(modelEditor);
    }
    // Simulation only
    else
    {
      acts[i]->setVisible(simulation);
      acts[i]->setEnabled(simulation);
    }
  }
}

/////////////////////////////////////////////////
void TopToolbar::InsertAction(const QString &_before, QAction *_action)
{
  QAction *beforeAction = this->dataPtr->toolbar->findChild<QAction *>(_before);
  if (!beforeAction)
  {
    gzerr << "Requested action [" << _before.toStdString() << "] not found"
          << std::endl;
    return;
  }

  this->dataPtr->toolbar->insertAction(beforeAction, _action);
}

/////////////////////////////////////////////////
QAction *TopToolbar::InsertSeparator(const QString &_before)
{
  QAction *beforeAction = this->dataPtr->toolbar->findChild<QAction *>(_before);
  if (!beforeAction)
  {
    gzerr << "Requested action [" << _before.toStdString() << "] not found"
          << std::endl;
    return NULL;
  }

  return this->dataPtr->toolbar->insertSeparator(beforeAction);
}

/////////////////////////////////////////////////
QAction *TopToolbar::InsertWidget(const QString &_before, QWidget *_widget)
{
  QAction *beforeAction = this->dataPtr->toolbar->findChild<QAction *>(_before);
  if (!beforeAction)
  {
    gzerr << "Requested action [" << _before.toStdString() << "] not found"
          << std::endl;
    return NULL;
  }

  return this->dataPtr->toolbar->insertWidget(beforeAction, _widget);
}
