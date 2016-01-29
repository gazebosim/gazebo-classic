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

const std::vector<std::string> PlotPalette::ModelProperties(
      {"Pose", "Velocity", "Acceleration", "Forces"});

/////////////////////////////////////////////////
PlotPalette::PlotPalette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PlotPalettePrivate)
{
  // Search
  auto searchEdit = new QLineEdit();
  this->connect(searchEdit, SIGNAL(textChanged(QString)), this,
      SLOT(UpdateSearch(QString)));

  this->dataPtr->searchModel = new QStandardItemModel();

  auto completer = new QCompleter(this->dataPtr->searchModel, this);
  completer->setCaseSensitivity(Qt::CaseInsensitive);
  searchEdit->setCompleter(completer);

  this->dataPtr->searchArea = new ConfigWidget();

  auto searchLayout = new QVBoxLayout();
  searchLayout->addWidget(searchEdit);

  auto searchWidget = new QWidget();
  searchWidget->setLayout(searchLayout);

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

  auto topicsSplitter = new QSplitter(Qt::Vertical, this);
  topicsSplitter->addWidget(topicsTopWidget);
  topicsSplitter->addWidget(topicsBottomWidget);
  topicsSplitter->setCollapsible(0, false);
  topicsSplitter->setCollapsible(1, false);

  QList<int> sizes;
  sizes << 50 << 50;
  topicsSplitter->setSizes(sizes);

  // Models top
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto configLayout = new QVBoxLayout();
  configLayout->addWidget(spacer);
  configLayout->setSpacing(0);

  this->dataPtr->modelsTop = new ConfigWidget();
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
  this->dataPtr->modelsBottom = new ConfigWidget();

  auto modelsBottomScroll = new QScrollArea;
  modelsBottomScroll->setWidget(this->dataPtr->modelsBottom);
  modelsBottomScroll->setWidgetResizable(true);

  auto modelsBottomLayout = new QVBoxLayout;
  modelsBottomLayout->setContentsMargins(0, 0, 0, 0);
  modelsBottomLayout->addWidget(modelsBottomScroll);

  auto modelsBottomWidget = new QWidget;
  modelsBottomWidget->setLayout(modelsBottomLayout);

  auto modelsSplitter = new QSplitter(Qt::Vertical, this);
  modelsSplitter->addWidget(modelsTopWidget);
  modelsSplitter->addWidget(modelsBottomWidget);
  modelsSplitter->setStretchFactor(0, 1);
  modelsSplitter->setStretchFactor(1, 2);
  modelsSplitter->setCollapsible(0, false);
  modelsSplitter->setCollapsible(1, false);

  // Sim top
  auto simTop = new ConfigWidget();

  // Sim bottom
  this->dataPtr->simBottom = new ConfigWidget();
  this->FillSimBottom();

  auto simBottomScroll = new QScrollArea;
  simBottomScroll->setWidget(this->dataPtr->simBottom);
  simBottomScroll->setWidgetResizable(true);

  auto simBottomLayout = new QVBoxLayout;
  simBottomLayout->setContentsMargins(0, 0, 0, 0);
  simBottomLayout->addWidget(simBottomScroll);

  auto simBottomWidget = new QWidget;
  simBottomWidget->setLayout(simBottomLayout);

  auto simSplitter = new QSplitter(Qt::Vertical, this);
  simSplitter->addWidget(simTop);
  simSplitter->addWidget(simBottomWidget);
  simSplitter->setStretchFactor(0, 1);
  simSplitter->setStretchFactor(1, 2);
  simSplitter->setCollapsible(0, false);
  simSplitter->setCollapsible(1, false);

  // Tabs
  auto tabWidget = new QTabWidget;
  tabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  tabWidget->setMinimumWidth(300);
  tabWidget->setStyleSheet("QTabWidget::pane{\
      border: none;\
      background-color: #303030}");

  tabWidget->addTab(topicsSplitter, "Topics");
  tabWidget->addTab(modelsSplitter, "Models");
  tabWidget->addTab(simSplitter, "Sim");
  tabWidget->addTab(searchWidget, "Search");

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
  configLayout->setSpacing(0);

  // Get all topics, independently of message type
  std::vector<std::string> topics;
  auto msgTypes = transport::getAdvertisedTopics();
  for (auto msgType : msgTypes)
  {
    for (auto topic : msgType.second)
    {
      topics.push_back(topic);
      this->dataPtr->searchModel->appendRow(new
          QStandardItem(QString::fromStdString(topic)));
    }
  }

  // Sort alphabetically
  std::sort(topics.begin(), topics.end());

  // Populate widget
  for (auto topic : topics)
  {
    // Shorten topic name
    auto shortName = topic;
    auto idX = shortName.find("/gazebo/default");
    if (idX != std::string::npos)
      shortName.replace(0, 15, "~");

    auto childWidget = new ItemConfigWidget(shortName);
    childWidget->SetPlotInfo(topic);
    this->connect(childWidget, SIGNAL(Clicked(const std::string &)), this,
        SLOT(OnTopicClicked(const std::string &)));

    this->dataPtr->topicsTop->AddConfigChildWidget(topic, childWidget);
    configLayout->addWidget(childWidget);
  }

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  configLayout->addWidget(spacer);

  this->dataPtr->topicsTop->setLayout(configLayout);
}

/////////////////////////////////////////////////
void PlotPalette::FillSimBottom()
{
  auto configLayout = new QVBoxLayout();
  configLayout->setSpacing(0);

  std::multimap<std::string, std::string> simFields = {
      {"~/world_stats", "sim_time"},
      {"~/world_stats", "real_time"},
      {"~/world_stats", "iterations"}};

  for (auto field : simFields)
  {
    auto humanName = ConfigWidget::HumanReadableKey(field.second);
    auto childWidget = new ItemConfigWidget(humanName);
    childWidget->SetDraggable(true);
    childWidget->SetPlotInfo(field.first + "::" + field.second);
    configLayout->addWidget(childWidget);
  }

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  configLayout->addWidget(spacer);

  this->dataPtr->simBottom->setLayout(configLayout);
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

  // Insert it on top
  auto vBoxLayout = qobject_cast<QVBoxLayout *>(
      this->dataPtr->modelsTop->layout());
  vBoxLayout->insertWidget(0, childWidget);
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
  // Clear previous layout
  auto oldLayout = this->dataPtr->topicsBottom->layout();
  if (oldLayout)
  {
    // Give ownership of all widgets to an object which will be out of scope
    QWidget().setLayout(oldLayout);
  }

  // Create a message from this topic
  auto msgType = transport::getTopicMsgType(_topic);
  if (msgType == "")
    return;

  auto msg = msgs::MsgFactory::NewMsg(msgType);

  // Create a new layout and fill it
  auto newLayout = new QVBoxLayout();
  newLayout->setSpacing(0);

  // Title
  auto title = new QLabel(QString::fromStdString(_topic));
  title->setMinimumHeight(40);
  title->setToolTip(tr((
      "<font size=3><p><b>Message type: </b>" + msgType + "</p></font>"
      ).c_str()));
  newLayout->addWidget(title);

  this->FillTopicFromMsg(msg.get(), _topic, 0, newLayout);

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  newLayout->addWidget(spacer);

  this->dataPtr->topicsBottom->setLayout(newLayout);
}

/////////////////////////////////////////////////
void PlotPalette::FillTopicFromMsg(google::protobuf::Message *_msg,
    const std::string &_scope, const unsigned int _level,
    QVBoxLayout *_parentLayout)
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
      case google::protobuf::FieldDescriptor::TYPE_FLOAT:
      case google::protobuf::FieldDescriptor::TYPE_INT64:
      case google::protobuf::FieldDescriptor::TYPE_UINT64:
      case google::protobuf::FieldDescriptor::TYPE_INT32:
      case google::protobuf::FieldDescriptor::TYPE_UINT32:
      case google::protobuf::FieldDescriptor::TYPE_BOOL:
      {
        auto humanName = ConfigWidget::HumanReadableKey(name);
        auto childWidget = new ItemConfigWidget(humanName, _level);
        childWidget->SetDraggable(true);
        childWidget->SetPlotInfo(_scope + "::" + name);

        std::string typeName = field->type_name();
        childWidget->setToolTip(tr((
            "<font size=3><p><b>Type: </b>" + typeName + "</p></font>"
            ).c_str()));

        _parentLayout->addWidget(childWidget);

        break;
      }
      // Message within a message
      case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
      {
        if (field->is_repeated())
          continue;

        auto fieldMsg = (ref->MutableMessage(_msg, field));

        if (field->message_type()->name() == "Time")
        {
          // We treat a full time message as double
          auto humanName = ConfigWidget::HumanReadableKey(name);
          auto childWidget = new ItemConfigWidget(humanName, _level);
          childWidget->SetDraggable(true);
          childWidget->SetPlotInfo(_scope + "::" + name);

          childWidget->setToolTip(tr(
              "<font size=3><p><b>Type:</b> double </p></font>"));

          _parentLayout->addWidget(childWidget);
        }
        else
        {
          // Create a collapsible list for submessages
          auto collapsibleLayout = new QVBoxLayout();
          collapsibleLayout->setContentsMargins(0, 0, 0, 0);
          collapsibleLayout->setSpacing(0);

          auto collapsibleWidget = new ConfigChildWidget();
          collapsibleWidget->setLayout(collapsibleLayout);

          auto groupWidget =
              this->dataPtr->topicsBottom->CreateGroupWidget(name,
              collapsibleWidget, _level);

          _parentLayout->addWidget(groupWidget);
          auto scope = _scope + "::" + name;

          this->FillTopicFromMsg(fieldMsg, scope, _level + 1,
              collapsibleLayout);
        }

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
  this->FillModel(_model);
}

/////////////////////////////////////////////////
void PlotPalette::FillModel(const std::string &_model)
{
  // Clear previous layout
  auto oldLayout = this->dataPtr->modelsBottom->layout();
  if (oldLayout)
  {
    // Give ownership of all widgets to an object which will be out of scope
    QWidget().setLayout(oldLayout);
  }

  // Fill new layout
  this->dataPtr->modelsBottom->setLayout(new QVBoxLayout());
  this->dataPtr->modelsBottom->layout()->setSpacing(0);

  for (auto prop : PlotPalette::ModelProperties)
  {
    auto childWidget = new ItemConfigWidget(prop);
    childWidget->SetDraggable(true);
    childWidget->SetPlotInfo(_model + "::" + prop);
    this->dataPtr->modelsBottom->layout()->addWidget(childWidget);
  }

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  this->dataPtr->modelsBottom->layout()->addWidget(spacer);
}

/////////////////////////////////////////////////
void PlotPalette::UpdateSearch(const QString &/*_search*/)
{
  // To be able to search all fields, we need to generate them all at first
  // and keep them in a list
}

/////////////////////////////////////////////////
ItemConfigWidget::ItemConfigWidget(const std::string &_text,
    const unsigned int _level)
  : ConfigChildWidget(), dataPtr(new ItemConfigWidgetPrivate)
{
  auto label = new QLabel(QString::fromStdString(_text));
  this->dataPtr->text = _text;
  this->dataPtr->draggable = false;

  auto mainLayout = new QHBoxLayout;
  if (_level != 0)
  {
    mainLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  mainLayout->addWidget(label);

  this->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);
  this->setFrameStyle(QFrame::Box);


  auto ss = ConfigWidget::StyleSheet("normal", _level);
  ss = ss + "QWidget:hover \
    {\
      color: " + ConfigWidget::greenColor + ";\
    }";
  this->setStyleSheet(ss);
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
  if (this->dataPtr->plotInfo == "")
    this->Clicked(this->Text());
  else
    this->Clicked(this->dataPtr->plotInfo);
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

/////////////////////////////////////////////////
void ItemConfigWidget::enterEvent(QEvent *_event)
{
  if (this->dataPtr->draggable)
   QApplication::setOverrideCursor(Qt::OpenHandCursor);
  else
   QApplication::setOverrideCursor(Qt::ArrowCursor);

  QWidget::enterEvent(_event);
}

/////////////////////////////////////////////////
void ItemConfigWidget::leaveEvent(QEvent *_event)
{
  if (this->dataPtr->draggable)
   QApplication::setOverrideCursor(Qt::ArrowCursor);

  QWidget::leaveEvent(_event);
}

