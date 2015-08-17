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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/TopToolbar.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TopToolbar::TopToolbar(QWidget *_parent) : QFrame(_parent)
{
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

  this->toolbar = new QToolBar;
  QHBoxLayout *toolLayout = new QHBoxLayout;
  toolLayout->setContentsMargins(0, 0, 0, 0);

  // Manipulation modes
  QActionGroup *actionGroup = new QActionGroup(this);
  if (g_arrowAct)
  {
    actionGroup->addAction(g_arrowAct);
    this->toolbar->addAction(g_arrowAct);
  }
  if (g_translateAct)
  {
    actionGroup->addAction(g_translateAct);
    this->toolbar->addAction(g_translateAct);
  }
  if (g_rotateAct)
  {
    actionGroup->addAction(g_rotateAct);
    this->toolbar->addAction(g_rotateAct);
  }
  if (g_scaleAct)
  {
    actionGroup->addAction(g_scaleAct);
    this->toolbar->addAction(g_scaleAct);
  }

  this->toolbar->addSeparator();

  // Insert simple shapes
  if (g_boxCreateAct)
    this->toolbar->addAction(g_boxCreateAct);
  if (g_sphereCreateAct)
    this->toolbar->addAction(g_sphereCreateAct);
  if (g_cylinderCreateAct)
    this->toolbar->addAction(g_cylinderCreateAct);
  this->toolbar->addSeparator();

  // Insert lights
  if (g_pointLghtCreateAct)
    this->toolbar->addAction(g_pointLghtCreateAct);
  if (g_spotLghtCreateAct)
    this->toolbar->addAction(g_spotLghtCreateAct);
  if (g_dirLghtCreateAct)
    this->toolbar->addAction(g_dirLghtCreateAct);
  this->toolbar->addSeparator();

  // Copy & Paste
  if (g_copyAct)
    this->toolbar->addAction(g_copyAct);
  if (g_pasteAct)
    this->toolbar->addAction(g_pasteAct);

  this->toolbar->addSeparator();

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
    g_alignButtonAct = this->toolbar->addWidget(alignButton);
    connect(alignButton, SIGNAL(pressed()), g_alignAct, SLOT(trigger()));
  }

  // Snap
  if (g_snapAct)
  {
    actionGroup->addAction(g_snapAct);
    this->toolbar->addAction(g_snapAct);
  }

  this->toolbar->addSeparator();

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
    g_viewAngleButtonAct = this->toolbar->addWidget(viewAngleButton);
  }

  // Empty space to push whatever comes next to the right
  QWidget *spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QAction *spacerAction = this->toolbar->addWidget(spacer);
  spacerAction->setObjectName("toolbarSpacerAction");

  // Screenshot / logging
  if (g_screenshotAct)
    this->toolbar->addAction(g_screenshotAct);
  if (g_dataLoggerAct)
    this->toolbar->addAction(g_dataLoggerAct);

  toolLayout->addSpacing(10);
  toolLayout->addWidget(this->toolbar);
  toolLayout->addSpacing(10);
  this->setLayout(toolLayout);
}

/////////////////////////////////////////////////
TopToolbar::~TopToolbar()
{
  delete this->toolbar;
  this->toolbar = NULL;
}

/////////////////////////////////////////////////
QToolBar *TopToolbar::GetToolbar() const
{
  return this->toolbar;
}
