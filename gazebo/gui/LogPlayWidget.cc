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
  this->setObjectName("logPlayWidget");

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
  {
    playToolbar->addAction(g_stepAct);
    g_stepAct->setEnabled(this->dataPtr->paused);
  }

  QLineEdit *simTimeEdit = new QLineEdit();
  simTimeEdit->setObjectName("logPlayWidgetSimTime");
  simTimeEdit->setReadOnly(true);
  simTimeEdit->setFixedWidth(110);

  QLineEdit *iterationsEdit = new QLineEdit();
  iterationsEdit->setReadOnly(true);
  iterationsEdit->setFixedWidth(110);
  iterationsEdit->setObjectName("logPlayWidgetIterations");

  QHBoxLayout *frameLayout = new QHBoxLayout;
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frameLayout->addItem(new QSpacerItem(5, -1, QSizePolicy::Expanding,
                             QSizePolicy::Minimum));
  frameLayout->addWidget(playToolbar);

  frameLayout->addWidget(new QLabel(tr("Sim Time:")));
  frameLayout->addWidget(simTimeEdit);

  frameLayout->addWidget(new QLabel(tr("Iterations:")));
  frameLayout->addWidget(iterationsEdit);

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
      iterationsEdit, SLOT(setText(QString)),
      Qt::QueuedConnection);

  // Create a QueuedConnection to set sim time.
  // This is used for thread safety.
  connect(this, SIGNAL(SetSimTime(QString)),
      simTimeEdit, SLOT(setText(QString)), Qt::QueuedConnection);
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
  this->dataPtr->paused = _paused;

  if (g_pauseAct)
    g_pauseAct->setVisible(!_paused);
  if (g_playAct)
    g_playAct->setVisible(_paused);
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
