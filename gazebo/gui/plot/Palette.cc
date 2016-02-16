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

#include "gazebo/common/Console.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/plot/Palette.hh"

#include "gazebo/transport/TransportIface.hh"

using namespace gazebo;
using namespace gui;

class TopicsViewDelegate : public QStyledItemDelegate
{
  public: enum DataRole
          {
            topicNameRole = Qt::UserRole + 100,
          };

  public: TopicsViewDelegate()
          {
          }

  public: virtual ~TopicsViewDelegate()
          {
          }

  public: void paint(QPainter *_painter, const QStyleOptionViewItem &_opt,
                     const QModelIndex &_index) const
          {
            QRectF r = _opt.rect;
            r.adjust(0, 5, 0, -5);
            _painter->setPen(QColor(30, 30, 30));

            QString topicName = qvariant_cast<QString>(
                _index.data(topicNameRole));
            _painter->drawText(r, topicName);
          }

  public: QSize sizeHint(const QStyleOptionViewItem &_option,
                         const QModelIndex &_index) const
          {
            QSize size = QStyledItemDelegate::sizeHint(_option, _index);
            QFont font = QApplication::font();
            QFontMetrics fm(font);
            size.setHeight(fm.height() + 10);
            return size;
          }
};

/// \brief Private data for the Palette class
class gazebo::gui::PalettePrivate
{
  /// \brief Top pane of the topics tab.
  public: QFrame *mainFrame;

  /// \brief Top pane of the topics tab.
  public: ConfigWidget *topicsTop;

  /// \brief Bottom pane of the topics tab.
  public: ConfigWidget *topicsBottom;

  /// \brief Bottom pane of the sim tab.
  public: ConfigWidget *simBottom;

  /// \brief Area for search results.
  public: ConfigWidget *searchArea;

  /// \brief Bottom pane of the search tab.
  public: ConfigWidget *searchBottom;

  /// \brief Keep all the information which can be searched.
  public: QStandardItemModel *searchModel;
};

/// \brief Private data for the PlotChildConfigWidget class
class gazebo::gui::PlotChildConfigWidgetPrivate
{
  /// \brief Text which is displayed on the widget.
  public: std::string text;

  /// \brief Information which is transmitted when this is dragged.
  /// It contains all the necessary information about the property to
  /// be plotted.
  public: std::string plotInfo;

  /// \brief Whether this is draggable or not.
  public: bool draggable;
};

/////////////////////////////////////////////////
Palette::Palette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PalettePrivate)
{
  // Search top
  /*auto searchEdit = new QLineEdit();
  this->connect(searchEdit, SIGNAL(textChanged(QString)), this,
      SLOT(UpdateSearch(QString)));

  this->dataPtr->searchModel = new QStandardItemModel();

  auto completer = new QCompleter(this->dataPtr->searchModel, this);
  completer->setCaseSensitivity(Qt::CaseInsensitive);
  searchEdit->setCompleter(completer);

  this->dataPtr->searchArea = new ConfigWidget();

  auto searchAreaScroll = new QScrollArea;
  searchAreaScroll->setWidget(this->dataPtr->searchArea);
  searchAreaScroll->setWidgetResizable(true);

  auto searchAreaLayout = new QVBoxLayout;
  searchAreaLayout->setContentsMargins(0, 0, 0, 0);
  searchAreaLayout->addWidget(searchAreaScroll);

  auto searchAreaWidget = new QWidget;
  searchAreaWidget->setLayout(searchAreaLayout);

  auto searchTopLayout = new QVBoxLayout();
  searchTopLayout->addWidget(searchEdit);
  searchTopLayout->addWidget(searchAreaWidget);

  auto searchTopWidget = new QWidget();
  searchTopWidget->setLayout(searchTopLayout);

  // Search bottom
  this->dataPtr->searchBottom = new ConfigWidget();

  auto searchBottomScroll = new QScrollArea;
  searchBottomScroll->setWidget(this->dataPtr->searchBottom);
  searchBottomScroll->setWidgetResizable(true);

  auto searchBottomLayout = new QVBoxLayout;
  searchBottomLayout->setContentsMargins(0, 0, 0, 0);
  searchBottomLayout->addWidget(searchBottomScroll);

  auto searchBottomWidget = new QWidget;
  searchBottomWidget->setLayout(searchBottomLayout);

  auto searchSplitter = new QSplitter(Qt::Vertical, this);
  searchSplitter->addWidget(searchTopWidget);
  searchSplitter->addWidget(searchBottomWidget);
  searchSplitter->setCollapsible(0, false);
  searchSplitter->setCollapsible(1, false);
  */

  QList<int> sizes;
  sizes << 50 << 50;
  //searchSplitter->setSizes(sizes);

  // Topics top
  /*this->dataPtr->topicsTop = new ConfigWidget();

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
  topicsSplitter->setSizes(sizes);

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
  */


  auto tabBar = new QTabBar;
  tabBar->addTab("Topics");
  tabBar->addTab("Sim");
  tabBar->addTab("Search");
  tabBar->setExpanding(true);
  tabBar->setDrawBase(false);

  QStandardItemModel *topicsModel = new QStandardItemModel;

  this->FillTopics(topicsModel);

  TopicsViewDelegate *topicsViewDelegate = new TopicsViewDelegate;

  QTreeView *topicsTree = new QTreeView;
  topicsTree->setObjectName("topicList");
  topicsTree->setAnimated(true);
  topicsTree->setHeaderHidden(true);
  topicsTree->setExpandsOnDoubleClick(true);
  topicsTree->setModel(topicsModel);
  topicsTree->setItemDelegate(topicsViewDelegate);
  topicsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);

  auto tabStackedLayout = new QStackedLayout;
  tabStackedLayout->setContentsMargins(0, 0, 0, 0);
  tabStackedLayout->addWidget(topicsTree);
  //tabStackedLayout->addWidget(simSplitter);
  //tabStackedLayout->addWidget(searchSplitter);

  connect(tabBar, SIGNAL(currentChanged(int)),
          tabStackedLayout, SLOT(setCurrentIndex(int)));

  auto mainFrameLayout = new QVBoxLayout;
  mainFrameLayout->addWidget(tabBar);
  mainFrameLayout->addLayout(tabStackedLayout);
  mainFrameLayout->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->mainFrame = new QFrame(this);
  this->dataPtr->mainFrame->setObjectName("plotPaletteFrame");
  this->dataPtr->mainFrame->setLayout(mainFrameLayout);

  /*auto collapseButton = new QPushButton("<");
  collapseButton->setObjectName("plotPaletteCollapse");
  connect(collapseButton, SIGNAL(clicked()), this, SLOT(OnCollapse()));

  auto collapseFiller = new QFrame;
  collapseFiller->setContentsMargins(0,0,0,0);
  collapseFiller->setObjectName("plotPaletteCollapseFiller");
  auto collapseFillerLayout = new QVBoxLayout;
  collapseFillerLayout->addWidget(collapseButton);
  collapseFillerLayout->addStretch(1);
  collapseFillerLayout->setContentsMargins(0, 0, 0, 0);
  collapseFiller->setLayout(collapseFillerLayout);
  */


  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(this->dataPtr->mainFrame);
  //mainLayout->addWidget(collapseFiller);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
Palette::~Palette()
{
}

/////////////////////////////////////////////////
void Palette::OnCollapse()
{
  this->dataPtr->mainFrame->setVisible(!this->dataPtr->mainFrame->isVisible());
}

/////////////////////////////////////////////////
void Palette::FillTopics(QStandardItemModel *_topicsModel)
{
  // Get all topics, independent of message type
  std::set<std::string> topics;
  QList<QStandardItem *> items;

  auto msgTopics = transport::getAdvertisedTopics();
  for (auto msgTopic : msgTopics)
  {
    for (auto topic : msgTopic.second)
    {
      topics.emplace(topic);
    }
  }

  // Populate widget
  for (auto topic : topics)
  {
    // Shorten topic name
    auto shortName = topic;
    auto idX = shortName.find("/gazebo/default");
    if (idX != std::string::npos)
      shortName.replace(0, 15, "~");

    QStandardItem *topicItem = new QStandardItem();
    topicItem->setData(shortName.c_str(), TopicsViewDelegate::topicNameRole);
    _topicsModel->appendRow(topicItem);

    // Create a message from this topic
    auto msgType = transport::getTopicMsgType(topic);
    if (msgType == "")
    {
      gzwarn << "Couldn't find message type for topic [" << _topic << "]"
        << std::endl;
      return;
    }

    auto msg = msgs::MsgFactory::NewMsg(msgType);
    this->FillFromMsg(msg.get(), topicItem);
    /*auto childWidget = new PlotChildConfigWidget(shortName);
    childWidget->SetPlotInfo(topic);
    this->connect(childWidget, SIGNAL(Clicked(const std::string &)), this,
        SLOT(OnTopicClicked(const std::string &)));

    configLayout->addWidget(childWidget);
    */
  }
}

/////////////////////////////////////////////////
void Palette::FillFromMsg(google::protobuf::Message *_msg,
    QStandardItem *_item);
    //const std::string &_scope, const unsigned int _level,
    //QVBoxLayout *_parentLayout)
{
  if (!_msg || !_item)
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
        // auto childWidget = new PlotChildConfigWidget(humanName, _level);
        //childWidget->SetDraggable(true);
        //childWidget->SetPlotInfo(_scope + "::" + name);

        std::string typeName = field->type_name();
        /*childWidget->setToolTip(tr((
            "<font size=3><p><b>Type: </b>" + typeName + "</p></font>"
            ).c_str()));
            */
          // Nate HERE
        auto *childItem = new QStandardItem();
        childItem->setData(humanName.c_str(),
            TopicsViewDelegate::topicNameRole);

        _item->appendRow(childItem)
        //_parentLayout->addWidget(childWidget);

        break;
      }
      // Message within a message
      case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
      {
        if (field->is_repeated())
          continue;

        auto fieldMsg = (ref->MutableMessage(_msg, field));

        // Treat time as double
        if (field->message_type()->name() == "Time")
        {
          auto humanName = ConfigWidget::HumanReadableKey(name);
          auto childWidget = new PlotChildConfigWidget(humanName, _level);
          childWidget->SetDraggable(true);
          childWidget->SetPlotInfo(_scope + "::" + name);

          childWidget->setToolTip(tr(
              "<font size=3><p><b>Type:</b> double </p></font>"));

          _parentLayout->addWidget(childWidget);
        }
        // Custom RPY widgets for orientation
        else if (field->message_type()->name() == "Quaternion")
        {
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

          std::vector<std::string> rpy = {"roll", "pitch", "yaw"};
          for (auto it : rpy)
          {
            auto humanName = ConfigWidget::HumanReadableKey(it);
            auto childWidget = new PlotChildConfigWidget(humanName, _level+1);
            childWidget->SetDraggable(true);
            childWidget->SetPlotInfo(_scope + "::" + name + "::" + it);

            childWidget->setToolTip(tr(
                "<font size=3><p><b>Type:</b> double </p></font>"));

            collapsibleLayout->addWidget(childWidget);
          }
        }
        // Create a collapsible list for submessages
        else
        {
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

          this->FillFromMsg(fieldMsg, scope, _level + 1,
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
/*void Palette::FillSimBottom()
{
  auto configLayout = new QVBoxLayout();
  configLayout->setSpacing(0);

  // Hard-coded values
  std::multimap<std::string, std::string> simFields = {
      {"~/world_stats", "sim_time"},
      {"~/world_stats", "real_time"},
      {"~/world_stats", "iterations"}};

  QList<QStandardItem *> items;
  for (auto field : simFields)
  {
    auto humanName = ConfigWidget::HumanReadableKey(field.second);
    auto childWidget = new PlotChildConfigWidget(humanName);
    childWidget->SetDraggable(true);
    childWidget->SetPlotInfo(field.first + "::" + field.second);
    configLayout->addWidget(childWidget);

    items.append(new QStandardItem(QString::fromStdString(humanName)));
  }

  // So they can be searched
  this->dataPtr->searchModel->insertColumn(1, items);

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  configLayout->addWidget(spacer);

  this->dataPtr->simBottom->setLayout(configLayout);
}

/////////////////////////////////////////////////
void Palette::OnTopicClicked(const std::string &_topic)
{
  this->FillTopicsBottom(_topic, this->dataPtr->topicsBottom);
}

/////////////////////////////////////////////////
void Palette::OnTopicSearchClicked(const std::string &_topic)
{
  this->FillTopicsBottom(_topic, this->dataPtr->searchBottom);
}*/

/////////////////////////////////////////////////
/*void Palette::FillTopicsBottom(const std::string &_topic,
    ConfigWidget *_widget)
{
  // Clear previous layout
  auto oldLayout = _widget->layout();
  if (oldLayout)
  {
    // Give ownership of all widgets to an object which will be out of scope
    QWidget().setLayout(oldLayout);
  }

  // Create a message from this topic
  auto msgType = transport::getTopicMsgType(_topic);
  if (msgType == "")
  {
    gzwarn << "Couldn't find message type for topic [" << _topic << "]"
        << std::endl;
    return;
  }

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

  this->FillFromMsg(msg.get(), _topic, 0, newLayout);

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  newLayout->addWidget(spacer);

  _widget->setLayout(newLayout);
}*/


/////////////////////////////////////////////////
/*void Palette::UpdateSearch(const QString &_search)
{
  // Clear previous layout
  auto oldLayout = this->dataPtr->searchArea->layout();
  if (oldLayout)
  {
    // Give ownership of all widgets to an object which will be out of scope
    QWidget().setLayout(oldLayout);
  }

  // Get all words in any order
  auto words = _search.split(" ");

  QString exp;
  if (!(words.size() == 1 && words[0] == ""))
  {
    exp = "^";
    for (auto word : words)
    {
      exp = exp + "(?=.*" + word + ")";
    }
    exp = exp + ".*$";
  }

  // New layout
  auto newLayout = new QVBoxLayout();
  newLayout->setSpacing(0);

  if (exp != "")
  {
    // Topics
    auto topics =
        this->dataPtr->searchModel->findItems(exp, Qt::MatchRegExp, 0);

    if (!topics.empty())
      newLayout->addWidget(new QLabel(tr("<b>Topics</b>")));

    for (auto topic : topics)
    {
      // TODO: highlight matched words
      // topic->text().replace(QRegExp(exp), "<b>\\1</b>");

      auto childWidget = new PlotChildConfigWidget(topic->text().toStdString());
      childWidget->SetPlotInfo(topic->text().toStdString());
      this->connect(childWidget, SIGNAL(Clicked(const std::string &)), this,
          SLOT(OnTopicSearchClicked(const std::string &)));

      newLayout->addWidget(childWidget);
    }

    // Sim
    auto sims = this->dataPtr->searchModel->findItems(exp, Qt::MatchRegExp, 1);

    if (!sims.empty())
      newLayout->addWidget(new QLabel(tr("<b>Simulation</b>")));

    for (auto sim : sims)
    {
      auto text = sim->text().toStdString();

      auto childWidget = new PlotChildConfigWidget(text);
      childWidget->SetDraggable(true);

      if (text.find("Sim") != std::string::npos)
        childWidget->SetPlotInfo("~/world_stats/sim_time");
      else if (text.find("Real") != std::string::npos)
        childWidget->SetPlotInfo("~/world_stats/real_time");
      else if (text.find("Itera") != std::string::npos)
        childWidget->SetPlotInfo("~/world_stats/iterations");

      newLayout->addWidget(childWidget);
    }
  }

  // Spacer
  auto spacer = new QWidget();
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  newLayout->addWidget(spacer);

  this->dataPtr->searchArea->setLayout(newLayout);
}*/

/////////////////////////////////////////////////
/*PlotChildConfigWidget::PlotChildConfigWidget(const std::string &_text,
    const unsigned int _level)
  : ConfigChildWidget(), dataPtr(new PlotChildConfigWidgetPrivate)
{
  this->dataPtr->text = _text;
  this->dataPtr->draggable = false;

  // Indentation
  auto mainLayout = new QHBoxLayout;
  if (_level != 0)
  {
    mainLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }

  // Text
  auto label = new QLabel(QString::fromStdString(_text));
  mainLayout->addWidget(label);

  // Style
  auto ss = ConfigWidget::StyleSheet("normal", _level);
  ss = ss + "QWidget:hover \
    {\
      color: " + ConfigWidget::greenColor + ";\
    }";
  this->setStyleSheet(ss);

  this->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);
  this->setFrameStyle(QFrame::Box);
}

/////////////////////////////////////////////////
PlotChildConfigWidget::~PlotChildConfigWidget()
{
}

/////////////////////////////////////////////////
std::string PlotChildConfigWidget::Text() const
{
  return this->dataPtr->text;
}

/////////////////////////////////////////////////
void PlotChildConfigWidget::SetDraggable(const bool _draggable)
{
  this->dataPtr->draggable = _draggable;
}

/////////////////////////////////////////////////
void PlotChildConfigWidget::SetPlotInfo(const std::string &_info)
{
  this->dataPtr->plotInfo = _info;
}

/////////////////////////////////////////////////
void PlotChildConfigWidget::mouseReleaseEvent(QMouseEvent *_event)
{
  if (this->dataPtr->plotInfo == "")
    this->Clicked(this->Text());
  else
    this->Clicked(this->dataPtr->plotInfo);
}

/////////////////////////////////////////////////
void PlotChildConfigWidget::mousePressEvent(QMouseEvent *_event)
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
void PlotChildConfigWidget::enterEvent(QEvent *_event)
{
  if (this->dataPtr->draggable)
    QApplication::setOverrideCursor(Qt::OpenHandCursor);
  else
    QApplication::setOverrideCursor(Qt::ArrowCursor);

  QWidget::enterEvent(_event);
}

/////////////////////////////////////////////////
void PlotChildConfigWidget::leaveEvent(QEvent *_event)
{
  if (this->dataPtr->draggable)
    QApplication::setOverrideCursor(Qt::ArrowCursor);

  QWidget::leaveEvent(_event);
}*/
