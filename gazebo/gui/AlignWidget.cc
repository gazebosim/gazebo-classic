/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/AlignWidgetPrivate.hh"
#include "gazebo/gui/AlignWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
AlignWidget::AlignWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new AlignWidgetPrivate)
{
  QLabel *xAlignLabel = new QLabel(tr("X: "));
  QLabel *yAlignLabel = new QLabel(tr("Y: "));
  QLabel *zAlignLabel = new QLabel(tr("Z: "));
  this->dataPtr->xAlignBar = new QToolBar(this);
  this->dataPtr->yAlignBar = new QToolBar(this);
  this->dataPtr->zAlignBar = new QToolBar(this);
  this->dataPtr->xAlignBar->addWidget(xAlignLabel);
  this->dataPtr->yAlignBar->addWidget(yAlignLabel);
  this->dataPtr->zAlignBar->addWidget(zAlignLabel);
  // align action triggered event
  this->dataPtr->alignSignalMapper = new QSignalMapper(this);
  connect(this->dataPtr->alignSignalMapper, SIGNAL(mapped(QString)),
      this, SLOT(OnAlignMode(QString)));

  QLabel *noteLabel = new QLabel(tr("Remember to Pause"));
  QFont labelFont = noteLabel->font();
  labelFont.setPointSize(labelFont.pointSize());
  noteLabel->setFont(labelFont);

  auto invertedBox = new QCheckBox("Reverse");
  connect(invertedBox, SIGNAL(toggled(bool)), this,
      SLOT(OnDirectionChanged(bool)));

  QComboBox *targetComboBox = new QComboBox(this);
  targetComboBox->addItem(tr("Relative to First"));
  targetComboBox->addItem(tr("Relative to Last"));
  connect(targetComboBox, SIGNAL(currentIndexChanged(int)), this,
      SLOT(OnAlignTargetChanged(int)));

  QVBoxLayout *alignLayout = new QVBoxLayout;
  alignLayout->addWidget(this->dataPtr->xAlignBar);
  alignLayout->addWidget(this->dataPtr->yAlignBar);
  alignLayout->addWidget(this->dataPtr->zAlignBar);
  alignLayout->addWidget(invertedBox);
  alignLayout->addWidget(targetComboBox);
  alignLayout->addWidget(noteLabel);
  alignLayout->setAlignment(this->dataPtr->xAlignBar, Qt::AlignHCenter);
  alignLayout->setAlignment(this->dataPtr->yAlignBar, Qt::AlignHCenter);
  alignLayout->setAlignment(this->dataPtr->zAlignBar, Qt::AlignHCenter);
  this->setLayout(alignLayout);

  this->dataPtr->alignRelativeTarget = 0;
}

/////////////////////////////////////////////////
AlignWidget::~AlignWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void AlignWidget::Add(AlignAxis _axis, AlignConfig _config, QAction *_action)
{
  std::string axisStr = this->GetAxisAsString(_axis);
  std::string configStr = this->GetConfigAsString(_config);
  if (_axis == ALIGN_X)
  {
    this->dataPtr->xAlignBar->addAction(_action);
    QWidget *xAlignWidget = this->dataPtr->xAlignBar->widgetForAction(_action);
    xAlignWidget->setProperty("axis", QVariant(tr(axisStr.c_str())));
    xAlignWidget->setProperty("config", QVariant(tr(configStr.c_str())));
    xAlignWidget->installEventFilter(this);
  }
  else if (_axis == ALIGN_Y)
  {
    this->dataPtr->yAlignBar->addAction(_action);
    QWidget *yAlignWidget = this->dataPtr->yAlignBar->widgetForAction(_action);
    yAlignWidget->setProperty("axis", QVariant(tr(axisStr.c_str())));
    yAlignWidget->setProperty("config", QVariant(tr(configStr.c_str())));
    yAlignWidget->installEventFilter(this);
  }
  else if (_axis == ALIGN_Z)
  {
    this->dataPtr->zAlignBar->addAction(_action);
    QWidget *zAlignWidget = this->dataPtr->zAlignBar->widgetForAction(_action);
    zAlignWidget->setProperty("axis", QVariant(tr(axisStr.c_str())));
    zAlignWidget->setProperty("config", QVariant(tr(configStr.c_str())));
    zAlignWidget->installEventFilter(this);
  }
  connect(_action, SIGNAL(triggered()),
      this->dataPtr->alignSignalMapper, SLOT(map()));
  this->dataPtr->alignSignalMapper->setMapping(
      _action, tr((axisStr+configStr).c_str()));
}

/////////////////////////////////////////////////
void AlignWidget::OnAlignMode(QString _mode)
{
  std::string mode = _mode.toStdString();
  gui::Events::alignMode(mode.substr(0, 1), mode.substr(1),
      this->dataPtr->alignRelativeTarget ? "last" : "first", false,
      this->dataPtr->inverted);
}

/////////////////////////////////////////////////
std::string AlignWidget::GetAxisAsString(AlignAxis _axis)
{
  if (_axis == ALIGN_X)
    return "x";
  else if (_axis == ALIGN_Y)
    return "y";
  else if (_axis == ALIGN_Z)
    return "z";
  return "";
}

/////////////////////////////////////////////////
std::string AlignWidget::GetConfigAsString(AlignConfig _config)
{
  if (_config == ALIGN_MIN)
    return "min";
  else if (_config == ALIGN_CENTER)
    return "center";
  else if (_config == ALIGN_MAX)
    return "max";
  return "";
}

/////////////////////////////////////////////////
void AlignWidget::OnAlignTargetChanged(int _index)
{
  this->dataPtr->alignRelativeTarget = _index;
}

/////////////////////////////////////////////////
void AlignWidget::OnDirectionChanged(bool _checked)
{
  this->dataPtr->inverted = _checked;
}

/////////////////////////////////////////////////
bool AlignWidget::eventFilter(QObject *_obj, QEvent *_event)
{
  if (this->isEnabled())
  {
    std::string axis = _obj->property("axis").toString().toStdString();
    std::string config = _obj->property("config").toString().toStdString();
    if (!config.empty() && !axis.empty())
    {
      if (_event->type() == QEvent::Enter)
      {
        gui::Events::alignMode(axis, config,
            this->dataPtr->alignRelativeTarget ? "last" : "first" , true,
            this->dataPtr->inverted);
      }
      else if (_event->type() == QEvent::Leave)
      {
        gui::Events::alignMode("", "reset",
            this->dataPtr->alignRelativeTarget ? "last" : "first", true,
            this->dataPtr->inverted);
      }
    }
  }
  return QWidget::eventFilter(_obj, _event);
}
