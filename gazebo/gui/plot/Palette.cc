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

/////////////////////////////////////////////////
/// \brief Delegate that handles drawing the topic tree
class PlotItemDelegate : public QStyledItemDelegate
{
  /// \brief The data roles
  public: enum DataRole
  {
    /// \brief Associated with the topic name
    TOPIC_NAME_ROLE = Qt::UserRole + 100,

    /// \brief Associated with the data name, this is used to pass
    /// information to a location during a drag-drop operation.
    DATA_ROLE,

    // \brief Data type name, used to display type information to the user.
    DATA_TYPE_NAME
  };

  /// \brief Constructor
  public: PlotItemDelegate() = default;

  /// \brief Destructor
  public: virtual ~PlotItemDelegate() = default;

  /// \brief Custom paint function.
  /// \param[in] _painter Pointer to the QT painter.
  /// \param[in] _opt Item options.
  /// \param[in] _index Item model index.
  public: void paint(QPainter *_painter, const QStyleOptionViewItem &_opt,
      const QModelIndex &_index) const
  {
    auto textRect = _opt.rect;

    // Custom options
    QString topicName = qvariant_cast<QString>(_index.data(TOPIC_NAME_ROLE));
    QString typeName = qvariant_cast<QString>(_index.data(DATA_TYPE_NAME));

    // TODO: Change to QApplication::font() once Roboto is used everywhere
    QFont font("Roboto Regular");
    QFontMetrics fm(font);

    // Handle hover style
    if (_opt.state & QStyle::State_MouseOver)
    {
      _painter->setPen(QPen(QColor(200, 200, 200, 0), 0));
      _painter->setBrush(QColor(200, 200, 200));
      _painter->drawRect(_opt.rect);
    }

    // Paint the icon, if present
    if (!typeName.isEmpty())
    {
      // Paint icon
      double iconSize = 20;

      QRectF iconRect = _opt.rect;
      iconRect.setTop(iconRect.top() + (_opt.rect.height()/2.0 - iconSize/2.0));

      QIcon icon(":/images/graph_line.svg");
      _painter->drawPixmap(iconRect.left(), iconRect.top(),
          icon.pixmap(iconSize, iconSize));

      // Move text
      textRect.adjust(iconSize + 5, 5, 0, -5);
    }
    else
    {
      textRect.adjust(0, 5, 0, -5);
    }

    _painter->setFont(QFont(font.family(), font.pointSize()));
    _painter->setPen(QColor(30, 30, 30));
    _painter->drawText(textRect, topicName);
  }

  /// \brief Size hint tells QT how big an item is.
  /// \param[in] _option Style options
  /// \param[in] _index Item model index
  public: QSize sizeHint(const QStyleOptionViewItem &_option,
                         const QModelIndex &_index) const
  {
    QSize size = QStyledItemDelegate::sizeHint(_option, _index);

    // TODO: Change to QApplication::font() once Roboto is used everywhere
    QFont font("Roboto Regular");
    QFontMetrics fm(font);

    // Make it slightly larger
    size.setHeight(fm.height() + 10);

    return size;
  }
};

/////////////////////////////////////////////////
/// Customize the item model so that we can pass along the correct MIME
/// information during a drag-drop.
class PlotItemModel : public QStandardItemModel
{
  /////////////////////////////////////////////////
  /// \brief Custom MIME data function.
  /// \param[in] _indexes List of selected items
  /// \return Mime data for the selected items.
  public: QMimeData *mimeData(const QModelIndexList &_indexes) const
  {
    QMimeData *curMimeData = new QMimeData();

    for (auto const &idx : _indexes)
    {
      if (idx.isValid())
      {
        QString text = this->data(idx,
            PlotItemDelegate::DATA_ROLE).toString();
        curMimeData->setData("application/x-item", text.toLatin1().data());

        break;
      }
    }

    return curMimeData;
  }
};

/////////////////////////////////////////////////
/// \brief Private data for the Palette class
class gazebo::gui::PalettePrivate
{
  /// \brief Model to hold topics data.
  public: PlotItemModel *topicsModel;

  /// \brief Model to hold sim data.
  public: PlotItemModel *simModel;
};

/////////////////////////////////////////////////
Palette::Palette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PalettePrivate)
{
  // The tab bar along the top.
  auto tabBar = new QTabBar;
  tabBar->setObjectName("plottingTabBar");
  tabBar->addTab("Topics");
  tabBar->addTab("Sim");
  tabBar->setExpanding(true);
  tabBar->setDrawBase(false);
  tabBar->setFocusPolicy(Qt::NoFocus);

  // Create a view delegate, to handle drawing items in the tree view
  auto plotItemDelegate = new PlotItemDelegate;

  // The model that will hold data to be displayed in the topic tree view
  this->dataPtr->topicsModel = new PlotItemModel;
  this->FillTopics(this->dataPtr->topicsModel);

  // A tree to visualize the topics and their messages.
  auto topicsTree = new QTreeView;
  topicsTree->setObjectName("topicList");
  topicsTree->setAnimated(true);
  topicsTree->setHeaderHidden(true);
  topicsTree->setExpandsOnDoubleClick(true);
  topicsTree->setModel(this->dataPtr->topicsModel);
  topicsTree->setItemDelegate(plotItemDelegate);
  topicsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  topicsTree->setDragEnabled(true);
  topicsTree->setDragDropMode(QAbstractItemView::DragOnly);

  // The model that will hold data to be displayed in the sim tree view
  this->dataPtr->simModel = new PlotItemModel;
  this->FillSim(this->dataPtr->simModel);

  // A tree to visualize sim variables
  auto simTree = new QTreeView;
  simTree->setObjectName("topicList");
  simTree->setAnimated(true);
  simTree->setHeaderHidden(true);
  simTree->setExpandsOnDoubleClick(true);
  simTree->setModel(this->dataPtr->simModel);
  simTree->setItemDelegate(plotItemDelegate);
  simTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  simTree->setDragEnabled(true);
  simTree->setDragDropMode(QAbstractItemView::DragOnly);

  // The stacked layout is used by the TabBar to switch active layouts
  auto tabStackedLayout = new QStackedLayout;
  tabStackedLayout->setContentsMargins(0, 0, 0, 0);
  tabStackedLayout->addWidget(topicsTree);
  tabStackedLayout->addWidget(simTree);

  // Connect TabBar to StackedLayout
  connect(tabBar, SIGNAL(currentChanged(int)),
          tabStackedLayout, SLOT(setCurrentIndex(int)));

  // Main frame
  auto mainFrameLayout = new QVBoxLayout;
  mainFrameLayout->addWidget(tabBar);
  mainFrameLayout->addLayout(tabStackedLayout);
  mainFrameLayout->setContentsMargins(0, 0, 0, 0);

  auto mainFrame = new QFrame(this);
  mainFrame->setObjectName("plotPaletteFrame");
  mainFrame->setLayout(mainFrameLayout);

  auto mainLayout = new QHBoxLayout;
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  this->setMinimumWidth(250);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
Palette::~Palette()
{
}

/////////////////////////////////////////////////
void Palette::FillTopics(QStandardItemModel *_topicsModel)
{
  // Get all topics, independent of message type
  std::set<std::string> topics;

  // Get all the unique topics
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

    auto topicItem = new QStandardItem();
    topicItem->setData(shortName.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
    _topicsModel->appendRow(topicItem);

    // Create a message from this topic to find out its fields
    auto msgType = transport::getTopicMsgType(topic);
    if (msgType == "")
    {
      gzwarn << "Couldn't find message type for topic [" << topic << "]"
        << std::endl;
      return;
    }

    auto msg = msgs::MsgFactory::NewMsg(msgType);
    this->FillFromMsg(msg.get(), topicItem, topic+"?p=");
  }
}

/////////////////////////////////////////////////
void Palette::FillSim(QStandardItemModel *_simModel)
{
  // Hard-coded values for the sim tab
  std::multimap<std::string, std::string> simFields = {
      {"~/world_stats", "sim_time"},
      {"~/world_stats", "real_time"},
      {"~/world_stats", "iterations"}};

  for (auto field : simFields)
  {
    auto childItem = new QStandardItem();
    childItem->setDragEnabled(true);

    auto humanName = ConfigWidget::HumanReadableKey(field.second);
    childItem->setData(humanName.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);

    if (field.second == "iterations")
      childItem->setData("Uint 64", PlotItemDelegate::DATA_TYPE_NAME);
    else
      childItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);

    std::string typeName =
        "<font size=3><p><b>Type</b>: " + childItem->data(
        PlotItemDelegate::DATA_TYPE_NAME).toString().toStdString() +
        "</p></font>";
    childItem->setToolTip(QString::fromStdString(typeName));

    std::string dataName = field.first + "?p=/" + field.second;
    childItem->setData(dataName.c_str(), PlotItemDelegate::DATA_ROLE);

    _simModel->appendRow(childItem);
  }
}

/////////////////////////////////////////////////
void Palette::FillFromMsg(google::protobuf::Message *_msg,
    QStandardItem *_item, const std::string &_uri)
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

        std::string typeName = field->type_name();

        auto *childItem = new QStandardItem();
        childItem->setData(humanName.c_str(),
            PlotItemDelegate::TOPIC_NAME_ROLE);

        switch (field->type())
        {
          default:
          case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
            childItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_FLOAT:
            childItem->setData("Float", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_INT64:
            childItem->setData("Int 64", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_UINT64:
            childItem->setData("Uint 64", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_INT32:
            childItem->setData("Int 32", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_UINT32:
            childItem->setData("Uint 32", PlotItemDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_BOOL:
            childItem->setData("Bool", PlotItemDelegate::DATA_TYPE_NAME);
            break;
        }
        childItem->setToolTip(
            "<font size=3><p><b>Type</b>: " + childItem->data(
            PlotItemDelegate::DATA_TYPE_NAME).toString() +
            "</p></font>");

        std::string dataName = _uri + "/" + name;
        childItem->setData(dataName.c_str(), PlotItemDelegate::DATA_ROLE);
        childItem->setDragEnabled(true);

        _item->appendRow(childItem);
        break;
      }
      // Message within a message
      case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
      {
        if (field->is_repeated())
          continue;

        // Treat time as double
        if (field->message_type()->name() == "Time")
        {
          auto humanName = ConfigWidget::HumanReadableKey(name);
          std::string dataName = _uri + "/" + name;

          auto *childItem = new QStandardItem();
          childItem->setData(humanName.c_str(),
              PlotItemDelegate::TOPIC_NAME_ROLE);
          childItem->setData(dataName.c_str(), PlotItemDelegate::DATA_ROLE);
          childItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
          childItem->setDragEnabled(true);
          childItem->setToolTip(
              "<font size=3><p><b>Type</b>: " + childItem->data(
              PlotItemDelegate::DATA_TYPE_NAME).toString() +
              "</p></font>");

          _item->appendRow(childItem);
        }
        // Custom XYZ widget for position
        else if (field->message_type()->name() == "Vector3d")
        {
          auto *posItem = new QStandardItem();
          posItem->setData(name.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
          _item->appendRow(posItem);

          std::vector<std::string> xyz = {"x", "y", "z"};
          for (auto it : xyz)
          {
            auto humanName = ConfigWidget::HumanReadableKey(it);
            std::string dataName = _uri + "/" + name + "/" + it;

            auto *childItem = new QStandardItem();
            childItem->setData(it.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
            childItem->setData(dataName.c_str(), PlotItemDelegate::DATA_ROLE);
            childItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);

            childItem->setToolTip(
                "<font size=3><p><b>Type</b>: " + childItem->data(
                PlotItemDelegate::DATA_TYPE_NAME).toString() +
                "</p></font>");
            childItem->setDragEnabled(true);

            posItem->appendRow(childItem);
          }
        }
        // Custom RPY widgets for orientation
        else if (field->message_type()->name() == "Quaternion")
        {
          auto *quatItem = new QStandardItem();
          quatItem->setData(name.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
          _item->appendRow(quatItem);

          std::vector<std::string> rpy = {"roll", "pitch", "yaw"};
          for (auto it : rpy)
          {
            auto humanName = ConfigWidget::HumanReadableKey(it);
            std::string dataName = _uri + "/" + name + "/" + it;
            auto *childItem = new QStandardItem();
            childItem->setData(it.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
            childItem->setData(dataName.c_str(), PlotItemDelegate::DATA_ROLE);
            childItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);

            childItem->setToolTip(
                "<font size=3><p><b>Type</b>: " + childItem->data(
                PlotItemDelegate::DATA_TYPE_NAME).toString() +
                "</p></font>");
            childItem->setDragEnabled(true);

            quatItem->appendRow(childItem);
          }
        }
        // Create a collapsible list for submessages
        else
        {
          auto fieldMsg = (ref->MutableMessage(_msg, field));
          auto *childItem = new QStandardItem();
          childItem->setData(name.c_str(), PlotItemDelegate::TOPIC_NAME_ROLE);
          _item->appendRow(childItem);
          this->FillFromMsg(fieldMsg, childItem, _uri + "/" + name);
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

