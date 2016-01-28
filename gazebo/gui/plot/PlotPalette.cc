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
  // Topics top
  this->dataPtr->topicsTop = new ConfigWidget();
  this->FillTopicsTop();

  auto topicsTopScroll = new QScrollArea;
  topicsTopScroll->setWidget(this->dataPtr->topicsTop);
  topicsTopScroll->setWidgetResizable(true);

  auto topicsTopLayout = new QVBoxLayout;
  topicsTopLayout->setContentsMargins(0, 0, 0, 0);
  topicsTopLayout->addWidget(topicsTopScroll);

  auto topicsTopWidget = new QWidget;
  topicsTopWidget->setLayout(topicsTopLayout);

  // Topics bottom
  this->dataPtr->topicsBottom = new ConfigWidget();

  auto topicsBottomScroll = new QScrollArea;
  topicsBottomScroll->setWidget(this->dataPtr->topicsBottom);
  topicsBottomScroll->setWidgetResizable(true);

  auto topicsBottomLayout = new QVBoxLayout;
  topicsBottomLayout->setContentsMargins(0, 0, 0, 0);
  topicsBottomLayout->addWidget(topicsBottomScroll);

  auto topicsBottomWidget = new QWidget;
  topicsBottomWidget->setLayout(topicsBottomLayout);

//  this->dataPtr->topicsBottom = new DragableListWidget(this);
//  this->dataPtr->topicsBottom->setDragEnabled(true);
//  this->dataPtr->topicsBottom->setDragDropMode(QAbstractItemView::DragOnly);
//  this->dataPtr->topicsBottom->setSizePolicy(
//      QSizePolicy::Minimum, QSizePolicy::Minimum);

  auto topicsSplitter = new QSplitter(Qt::Vertical, this);
  topicsSplitter->addWidget(topicsTopWidget);
  topicsSplitter->addWidget(topicsBottomWidget);
  topicsSplitter->setStretchFactor(0, 1);
  topicsSplitter->setStretchFactor(1, 1);
  topicsSplitter->setCollapsible(0, false);
  topicsSplitter->setCollapsible(1, false);

  // Models top
  this->dataPtr->modelsTop = new ConfigWidget();
  auto configLayout = new QVBoxLayout();
  configLayout->setContentsMargins(0, 0, 0, 0);
  configLayout->setSpacing(0);
  this->dataPtr->modelsTop->setLayout(configLayout);

  auto modelsScroll = new QScrollArea;
  modelsScroll->setWidget(this->dataPtr->modelsTop);
  modelsScroll->setWidgetResizable(true);

  auto modelsLayout = new QVBoxLayout;
  modelsLayout->setContentsMargins(0, 0, 0, 0);
  modelsLayout->addWidget(modelsScroll);

  auto modelsTopWidget = new QWidget;
  modelsTopWidget->setLayout(modelsLayout);

  this->connect(this, SIGNAL(InsertModelSignal(const std::string &)), this,
      SLOT(OnInsertModelSignal(const std::string &)));
  this->connect(this, SIGNAL(RemoveModelSignal(const std::string &)), this,
      SLOT(OnRemoveModelSignal(const std::string &)));

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
//  for (int i = 0; i < 10; ++i)
//  {
//    auto item = new QListWidgetItem("Topic prop " + QString::number(i));
//    this->dataPtr->topicsBottom->addItem(item);
//  }

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
  tabWidget->setMinimumWidth(300);

  tabWidget->addTab(topicsSplitter, "Topics");
  tabWidget->addTab(modelsSplitter, "Models");
  tabWidget->addTab(simSplitter, "Sim");
  tabWidget->addTab(new QWidget(), "Search");

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(tabWidget);

  this->setLayout(mainLayout);

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->newModelSub = this->dataPtr->node->Subscribe("~/model/info",
      &PlotPalette::OnModel, this, true);

  this->dataPtr->requestPub =
      this->dataPtr->node->Advertise<msgs::Request>("~/request");
  this->dataPtr->responseSub = this->dataPtr->node->Subscribe("~/response",
      &PlotPalette::OnResponse, this);

  this->dataPtr->requestMsg = msgs::CreateRequest("scene_info");
  this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &PlotPalette::OnRequest, this, false);
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
    this->connect(childWidget, SIGNAL(Clicked(const std::string &)), this,
        SLOT(OnTopicClicked(const std::string &)));

    this->dataPtr->topicsTop->AddConfigChildWidget(topic, childWidget);
    configLayout->addWidget(childWidget);
  }

  this->dataPtr->topicsTop->setLayout(configLayout);
}

/////////////////////////////////////////////////
void PlotPalette::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    this->RemoveModelSignal(_msg->data());
  }
}

/////////////////////////////////////////////////
void PlotPalette::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->dataPtr->requestMsg || _msg->id() !=
      this->dataPtr->requestMsg->id())
  {
    return;
  }

  msgs::Scene sceneMsg;

  if (_msg->has_type() && _msg->type() == sceneMsg.GetTypeName())
  {
    sceneMsg.ParseFromString(_msg->serialized_data());

    for (int i = 0; i < sceneMsg.model_size(); ++i)
    {
      this->InsertModelSignal(sceneMsg.model(i).name());
    }
  }

  delete this->dataPtr->requestMsg;
  this->dataPtr->requestMsg = NULL;
}

/////////////////////////////////////////////////
void PlotPalette::OnModel(ConstModelPtr &_msg)
{
  this->InsertModelSignal(_msg->name());
}

/////////////////////////////////////////////////
void PlotPalette::OnInsertModelSignal(const std::string &_name)
{
  auto childWidget = new ItemConfigWidget(_name);
  this->connect(childWidget, SIGNAL(Clicked(const std::string &)), this,
      SLOT(OnModelClicked(const std::string &)));

  this->dataPtr->modelsTop->AddConfigChildWidget(_name, childWidget);

  this->dataPtr->modelsTop->layout()->addWidget(childWidget);
}

/////////////////////////////////////////////////
void PlotPalette::OnRemoveModelSignal(const std::string &_name)
{
  auto models = this->dataPtr->modelsTop->findChildren<ItemConfigWidget *>();

  for (auto model : models)
  {
    if (model->Text() == _name)
    {
      this->dataPtr->modelsTop->layout()->removeWidget(model);
      delete model;
      return;
    }
  }
}

/////////////////////////////////////////////////
void PlotPalette::OnTopicClicked(const std::string &_topic)
{
  // Create a message from this topic
  auto msgType = transport::getTopicMsgType(_topic);
  if (msgType == "")
    return;

  auto msg = msgs::MsgFactory::NewMsg(msgType);

  // Create a new layout and fill it
  auto newLayout = new QVBoxLayout();
  this->FillTopicFromMsg(msg.get(), _topic, newLayout);

  // Clear previous layout and use new one
  auto oldLayout = this->dataPtr->topicsBottom->layout();
  if (oldLayout)
  {
    // Give ownership of all widgets to an object which will be out of scope
    QWidget().setLayout(oldLayout);
  }

  this->dataPtr->topicsBottom->setLayout(newLayout);
}

/////////////////////////////////////////////////
void PlotPalette::FillTopicFromMsg(google::protobuf::Message *_msg,
    const std::string &_scope, QVBoxLayout *_parentLayout)
{
  if (!_parentLayout)
    return;

  auto ref = _msg->GetReflection();
  if (!ref)
    return;

  auto descriptor = _msg->GetDescriptor();
  if (!descriptor)
    return;

  auto count = descriptor->field_count();

  // Go through all fields in this message
  for (int i = 0; i < count; ++i)
  {
    auto field = descriptor->field(i);
    if (!field)
      return;

    auto name = field->name();

    switch (field->type())
    {
      case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
      {
        auto childWidget = new ItemConfigWidget(name);
        childWidget->SetDraggable(true);
        childWidget->SetPlotInfo(_scope + ":" + name);
        _parentLayout->addWidget(childWidget);

        break;
      }
      // Message within a message
      case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
      {
        if (field->is_repeated())
          continue;

        auto fieldMsg = (ref->MutableMessage(_msg, field));

        // Create a collapsible list for submessages
        _parentLayout->addWidget(new QLabel(QString::fromStdString(name)));
        auto scope = _scope + ":" + name;

        auto innerLayout = new QVBoxLayout();
        _parentLayout->addLayout(innerLayout);
        this->FillTopicFromMsg(fieldMsg, scope, innerLayout);

        break;
      }
      default:
      {
        continue;
      }
    }
  }
}

/////////////////////////////////////////////////
void PlotPalette::OnModelClicked(const std::string &_model)
{
  gzdbg << "Fill bottom with props for [" << _model << "]" << std::endl;
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
  : ConfigChildWidget(), dataPtr(new ItemConfigWidgetPrivate)
{
  auto label = new QLabel(QString::fromStdString(_text));
  this->dataPtr->text = _text;
  this->dataPtr->draggable = false;

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
std::string ItemConfigWidget::Text() const
{
  return this->dataPtr->text;
}

/////////////////////////////////////////////////
void ItemConfigWidget::SetDraggable(const bool _draggable)
{
  this->dataPtr->draggable = _draggable;
}

/////////////////////////////////////////////////
void ItemConfigWidget::SetPlotInfo(const std::string &_info)
{
  this->dataPtr->plotInfo = _info;
}

/////////////////////////////////////////////////
void ItemConfigWidget::mouseReleaseEvent(QMouseEvent */*_event*/)
{
/*  auto ss = ConfigWidget::StyleSheet("normal");
  ss = ss + "QWidget \
    {\
      color: " + ConfigWidget::blueColor + ";\
    }";
  this->setStyleSheet(ss);
*/
  this->Clicked(this->Text());
}

/////////////////////////////////////////////////
void ItemConfigWidget::mousePressEvent(QMouseEvent *_event)
{
  if (_event->button() == Qt::LeftButton &&
      this->dataPtr->draggable)
  {
    auto currMimeData = new QMimeData;
    QByteArray ba;
    ba = QString::fromStdString(this->dataPtr->plotInfo).toLatin1().data();
    currMimeData->setData("application/x-item", ba);
    auto drag = new QDrag(this);
    drag->setMimeData(currMimeData);
    drag->exec(Qt::LinkAction);
  }
  QWidget::mousePressEvent(_event);
}

