/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/plot/PlotPalette.hh"
#include "gazebo/gui/plot/PlotPalettePrivate.hh"

#include "gazebo/transport/TransportIface.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PlotPalette::PlotPalette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PlotPalettePrivate)
{
  // Topics
  this->dataPtr->topicsTop = new ConfigWidget();
  this->FillTopicsTop();

  auto topicsScroll = new QScrollArea;
  topicsScroll->setWidget(this->dataPtr->topicsTop);
  topicsScroll->setWidgetResizable(true);

  auto topicsLayout = new QVBoxLayout;
  topicsLayout->setContentsMargins(0, 0, 0, 0);
  topicsLayout->addWidget(topicsScroll);

  auto topicsTopWidget = new QWidget;
  topicsTopWidget->setLayout(topicsLayout);

  this->dataPtr->topicsBottom = new DragableListWidget(this);
  this->dataPtr->topicsBottom->setDragEnabled(true);
  this->dataPtr->topicsBottom->setDragDropMode(QAbstractItemView::DragOnly);
  this->dataPtr->topicsBottom->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);

  auto topicsSplitter = new QSplitter(Qt::Vertical, this);
  topicsSplitter->addWidget(topicsTopWidget);
  topicsSplitter->addWidget(this->dataPtr->topicsBottom);
  topicsSplitter->setStretchFactor(0, 1);
  topicsSplitter->setStretchFactor(1, 2);
  topicsSplitter->setCollapsible(0, false);
  topicsSplitter->setCollapsible(1, false);

  // Models top
  this->dataPtr->modelsTop = new ConfigWidget();
  this->FillModelsTop();

  auto modelsScroll = new QScrollArea;
  modelsScroll->setWidget(this->dataPtr->modelsTop);
  modelsScroll->setWidgetResizable(true);

  auto modelsLayout = new QVBoxLayout;
  modelsLayout->setContentsMargins(0, 0, 0, 0);
  modelsLayout->addWidget(modelsScroll);

  auto modelsTopWidget = new QWidget;
  modelsTopWidget->setLayout(modelsLayout);

  // Models bottom
  this->dataPtr->modelsBottom = new DragableListWidget(this);
  this->dataPtr->modelsBottom->setDragEnabled(true);
  this->dataPtr->modelsBottom->setDragDropMode(QAbstractItemView::DragOnly);
  this->dataPtr->modelsBottom->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);

  auto modelsSplitter = new QSplitter(Qt::Vertical, this);
  modelsSplitter->addWidget(modelsTopWidget);
  modelsSplitter->addWidget(this->dataPtr->modelsBottom);
  modelsSplitter->setStretchFactor(0, 1);
  modelsSplitter->setStretchFactor(1, 2);
  modelsSplitter->setCollapsible(0, false);
  modelsSplitter->setCollapsible(1, false);

  // Sim
  auto simTop = new ConfigWidget();

  this->dataPtr->simBottom = new DragableListWidget(this);
  this->dataPtr->simBottom->setDragEnabled(true);
  this->dataPtr->simBottom->setDragDropMode(QAbstractItemView::DragOnly);
  this->dataPtr->simBottom->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);

  auto simSplitter = new QSplitter(Qt::Vertical, this);
  simSplitter->addWidget(simTop);
  simSplitter->addWidget(this->dataPtr->simBottom);
  simSplitter->setStretchFactor(0, 1);
  simSplitter->setStretchFactor(1, 2);
  simSplitter->setCollapsible(0, false);
  simSplitter->setCollapsible(1, false);

  //=================
  // TODO for testing - remove later
  for (int i = 0; i < 10; ++i)
  {
    auto item = new QListWidgetItem("Topic prop " + QString::number(i));
    this->dataPtr->topicsBottom->addItem(item);
  }

  for (int i = 0; i < 10; ++i)
  {
    auto item = new QListWidgetItem("Model prop " + QString::number(i));
    this->dataPtr->modelsBottom->addItem(item);
  }

  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  item->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->simBottom->addItem(item);

  QListWidgetItem *itema = new QListWidgetItem("Dog");
  itema->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->simBottom->addItem(itema);

  QListWidgetItem *itemb = new QListWidgetItem("Cat");
  itemb->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->simBottom->addItem(itemb);

  QListWidgetItem *itemc = new QListWidgetItem("Turtle");
  itemc->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->simBottom->addItem(itemc);

  //=================


  // Tabs
  auto tabWidget = new QTabWidget;
  tabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  tabWidget->setMinimumWidth(250);

  tabWidget->addTab(topicsSplitter, "Topics");
  tabWidget->addTab(modelsSplitter, "Models");
  tabWidget->addTab(simSplitter, "Sim");

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(tabWidget);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
PlotPalette::~PlotPalette()
{
}

/////////////////////////////////////////////////
void PlotPalette::FillTopicsTop()
{
  auto configLayout = new QVBoxLayout();
  configLayout->setContentsMargins(0, 0, 0, 0);
  configLayout->setSpacing(0);

  // Get all topics, independently of message type
  std::vector<std::string> topics;
  auto msgTypes = transport::getAdvertisedTopics();
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      topics.push_back(topic);
    }
  }

  // Sort alphabetically
  std::sort(topics.begin(), topics.end());

  // Populate widget
  for (auto topic : topics)
  {
    auto childWidget = new ItemConfigWidget(topic);
   // connect(childWidget, SIGNAL(Clicked()), this, SLOT(OnTopicClicked()));

    this->dataPtr->topicsTop->AddConfigChildWidget(topic, childWidget);
    configLayout->addWidget(childWidget);
  }

  this->dataPtr->topicsTop->setLayout(configLayout);
}

/////////////////////////////////////////////////
void PlotPalette::FillModelsTop()
{
  auto configLayout = new QVBoxLayout();
  configLayout->setContentsMargins(0, 0, 0, 0);
  configLayout->setSpacing(0);
  for (int i = 0; i < 10; ++i)
  {
    auto childWidget = new ItemConfigWidget("Model " + std::to_string(i));
    connect(childWidget, SIGNAL(Clicked()), this, SLOT(OnModelClicked()));

    this->dataPtr->modelsTop->AddConfigChildWidget("Model " + i, childWidget);

    configLayout->addWidget(childWidget);
  }
  this->dataPtr->modelsTop->setLayout(configLayout);
}

/////////////////////////////////////////////////
void PlotPalette::OnModelClicked()
{
  // Fill modelBottom with properties of clicked model
}

/////////////////////////////////////////////////
DragableListWidget::DragableListWidget(QWidget *_parent) : QListWidget(_parent)
{
}

/////////////////////////////////////////////////
void DragableListWidget::startDrag(Qt::DropActions /*_supportedActions*/)
{
  QListWidgetItem *currItem = this->currentItem();
  QMimeData *currMimeData = new QMimeData;
  QByteArray ba;
  ba = currItem->text().toLatin1().data();
  currMimeData->setData("application/x-item", ba);
  QDrag *drag = new QDrag(this);
  drag->setMimeData(currMimeData);
  drag->exec(Qt::LinkAction);
}

/////////////////////////////////////////////////
Qt::DropActions DragableListWidget::supportedDropActions()
{
  return Qt::LinkAction;
}

/////////////////////////////////////////////////
ItemConfigWidget::ItemConfigWidget(const std::string &_text)
  : ConfigChildWidget()
{
  auto label = new QLabel(QString::fromStdString(_text));

  auto mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(label);

  this->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);
  this->setFrameStyle(QFrame::Box);


  auto ss = ConfigWidget::StyleSheet("normal");
  ss = ss + "QWidget:hover \
    {\
      color: " + ConfigWidget::greenColor + ";\
    }";
  this->setStyleSheet(ss);
  this->setMinimumHeight(25);
  this->setMaximumHeight(25);
}

/////////////////////////////////////////////////
ItemConfigWidget::~ItemConfigWidget()
{
}

/////////////////////////////////////////////////
void ItemConfigWidget::mouseReleaseEvent(QMouseEvent */*_event*/)
{
  auto ss = ConfigWidget::StyleSheet("normal");
  ss = ss + "QWidget \
    {\
      color: " + ConfigWidget::blueColor + ";\
    }";
  this->setStyleSheet(ss);

  this->Clicked();
}

