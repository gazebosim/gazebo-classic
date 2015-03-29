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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/LogPlayWidget.hh"
#include "gazebo/gui/LogPlayWidgetPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LogPlayWidget::LogPlayWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new LogPlayWidgetPrivate)
{
  this->setObjectName("timePanel");

  this->dataPtr->timePanel = dynamic_cast<TimePanel *>(_parent);

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QScrollArea *scrollArea = new QScrollArea(this);
  scrollArea->setLineWidth(1);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setFrameShadow(QFrame::Plain);
  scrollArea->setWidgetResizable(true);
  scrollArea->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  // Play control (Play/Step/Pause)
  QSpinBox *stepSpinBox = new QSpinBox;
  stepSpinBox->setRange(1, 9999);
  connect(stepSpinBox, SIGNAL(valueChanged(int)), this,
      SLOT(OnStepValueChanged(int)));

  QWidget *stepWidget = new QWidget;
  QLabel *stepLabel = new QLabel(tr("Steps:"));
  QVBoxLayout *stepLayout = new QVBoxLayout;
  stepLayout->addWidget(stepLabel);
  stepLayout->addWidget(stepSpinBox);
  stepWidget->setLayout(stepLayout);

  QLabel *stepToolBarLabel = new QLabel(tr("Steps:"));

  QMenu *stepMenu = new QMenu;
  this->dataPtr->stepButton = new QToolButton;
  this->dataPtr->stepButton->setMaximumSize(35, this->dataPtr->stepButton->height());
  QWidgetAction *stepAction = new QWidgetAction(stepMenu);
  stepAction->setDefaultWidget(stepWidget);
  stepMenu->addAction(stepAction);
  this->dataPtr->stepButton->setMenu(stepMenu);
  this->dataPtr->stepButton->setPopupMode(QToolButton::InstantPopup);
  this->dataPtr->stepButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
  this->dataPtr->stepButton->setContentsMargins(0, 0, 0, 0);
  this->OnStepValueChanged(1);

  connect(stepSpinBox, SIGNAL(editingFinished()), stepMenu,
      SLOT(hide()));

  QFrame *frame = new QFrame(scrollArea);
  frame->setFrameShape(QFrame::NoFrame);
  scrollArea->setWidget(frame);

  QToolBar *playToolbar = new QToolBar;
  playToolbar->setObjectName("playToolBar");
  playToolbar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  if (g_playAct)
    playToolbar->addAction(g_playAct);
  if (g_pauseAct)
    playToolbar->addAction(g_pauseAct);
  this->dataPtr->paused = false;

  QLabel *emptyLabel = new QLabel(tr("  "));
  playToolbar->addWidget(emptyLabel);

  if (g_stepAct)
    playToolbar->addAction(g_stepAct);
  this->dataPtr->stepToolBarLabelAction = playToolbar->addWidget(stepToolBarLabel);
  this->dataPtr->stepButtonAction = playToolbar->addWidget(this->dataPtr->stepButton);
  this->dataPtr->stepButtonAction->setObjectName("timePanelStepAction");

  this->dataPtr->simTimeEdit = new QLineEdit;
  this->dataPtr->simTimeEdit->setObjectName("timePanelSimTime");
  this->dataPtr->simTimeEdit->setReadOnly(true);
  this->dataPtr->simTimeEdit->setFixedWidth(110);

  this->dataPtr->iterationsEdit = new QLineEdit;
  this->dataPtr->iterationsEdit->setReadOnly(true);
  this->dataPtr->iterationsEdit->setFixedWidth(110);
  this->dataPtr->iterationsEdit->setObjectName("timePanelIterations");

  QHBoxLayout *frameLayout = new QHBoxLayout;
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frameLayout->addItem(new QSpacerItem(5, -1, QSizePolicy::Expanding,
                             QSizePolicy::Minimum));
  frameLayout->addWidget(playToolbar);

  this->dataPtr->simTimeLabel = new QLabel(tr("Sim Time:"));
  frameLayout->addWidget(this->dataPtr->simTimeLabel);
  frameLayout->addWidget(this->dataPtr->simTimeEdit);

  this->dataPtr->iterationsLabel = new QLabel(tr("Iterations:"));
  frameLayout->addWidget(this->dataPtr->iterationsLabel);
  frameLayout->addWidget(this->dataPtr->iterationsEdit);

  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(scrollArea);
  mainLayout->setAlignment(Qt::AlignCenter);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // Create a QueuedConnection to set iterations.
  // This is used for thread safety.
  connect(this, SIGNAL(SetIterations(QString)),
          this->dataPtr->iterationsEdit, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set sim time.
  // This is used for thread safety.
  connect(this, SIGNAL(SetSimTime(QString)),
          this->dataPtr->simTimeEdit, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
bool LogPlayWidget::IsPaused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
void LogPlayWidget::SetPaused(bool _paused)
{
  if (g_pauseAct)
    g_pauseAct->setVisible(!_paused);
  if (g_playAct)
    g_playAct->setVisible(_paused);

  this->dataPtr->paused = _paused;
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStepValueChanged(int _value)
{
  // text formating and resizing for better presentation
  std::string numStr = QString::number(_value).toStdString();
  QFont stepFont = this->dataPtr->stepButton->font();
  stepFont.setPointSizeF(11 - numStr.size()/2.0);
  this->dataPtr->stepButton->setFont(stepFont);
  numStr.insert(numStr.end(), 4 - numStr.size(), ' ');
  this->dataPtr->stepButton->setText(tr(numStr.c_str()));

  this->dataPtr->timePanel->OnStepValueChanged(_value);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetSimTime(QString _time)
{
  this->SetSimTime(_time);
}

/////////////////////////////////////////////////
void LogPlayWidget::EmitSetIterations(QString _time)
{
  this->SetIterations(_time);
}
