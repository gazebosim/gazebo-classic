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

#include <set>
#include <string>
#include <google/protobuf/message.h>

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
/// Customize the search model.
class SearchModel : public QSortFilterProxyModel
{
  /////////////////////////////////////////////////
  /// \brief Customize so we accept rows where:
  /// 1. Each of the words can be found in its ancestors or itself, but not
  /// necessarily all words on the same row, or
  /// 2. One of its descendants matches rule 1, or
  /// 3. One of its parents matches rule 1.
  ///
  /// For example this structure:
  /// - a
  /// -- b
  /// -- c
  /// --- d
  ///
  /// * A search of "a" will display all rows.
  /// * A search of "b" or "a b" will display "a" and "b".
  /// * A search of "c", "d", "a c", "a d", "a c d" or "c d" will display "a",
  /// "c" and "d".
  /// * A search of "a b c d", "b c" or "b d" will display nothing.
  ///
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if row is accepted.
  public: bool filterAcceptsRow(int _srcRow, const QModelIndex &_srcParent)
      const
  {
    auto words = this->search.split(" ");

    // Each word must match at least once, either self, parent or child
    for (auto word : words)
    {
      // Row itself contains this word
      if (this->filterAcceptsRowItself(_srcRow, _srcParent, word))
        continue;

      // One of the ancestors contains this word
      QModelIndex parentIndex = _srcParent;
      bool parentAccepted = false;
      while (parentIndex.isValid())
      {
        if (this->filterAcceptsRowItself(parentIndex.row(),
            parentIndex.parent(), word))
        {
          parentAccepted = true;
          break;
        }
        parentIndex = parentIndex.parent();
      }

      if (parentAccepted)
        continue;

      // At least one of the children fits rule 1
      if (this->hasAcceptedChildren(_srcRow, _srcParent))
        continue;

      // This word can't be found on the row or a parent, and no child is fully
      // accepted.
      return false;
    }

    return true;
  }

  /// \brief Check if row contains the word on itself.
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if row matches.
  public: bool filterAcceptsRowItself(const int _srcRow, const
      QModelIndex &_srcParent, QString _word) const
  {
    auto id = this->sourceModel()->index(_srcRow, 0, _srcParent);

    return (this->sourceModel()->data(id,
        this->filterRole()).toString().contains(_word, Qt::CaseInsensitive));
  }

  /// \brief Check if any of the children is fully accepted.
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if any of the children match.
  public: bool hasAcceptedChildren(const int _srcRow,
      const QModelIndex &_srcParent) const
  {
    QModelIndex item = sourceModel()->index(_srcRow, 0, _srcParent);

    if (!item.isValid())
      return false;

    for (int i = 0; i < item.model()->rowCount(item); ++i)
    {
      if (this->filterAcceptsRow(i, item))
        return true;
    }

    return false;
  }

  /// \brief Set a new search value.
  /// \param[in] _search Full search string.
  public: void setSearch(const QString &_search)
  {
    this->search = _search;
    this->filterChanged();
  }

  /// \brief Full search string.
  public: QString search;
};

/////////////////////////////////////////////////
/// \brief Private data for the Palette class
class gazebo::gui::PalettePrivate
{
  /// \brief Model to hold topics data.
  public: PlotItemModel *topicsModel;

  /// \brief Model to hold sim data.
  public: PlotItemModel *simModel;

  /// \brief Proxy model to filter topics data.
  public: SearchModel *searchTopicsModel;

  /// \brief Proxy model to filter sim data.
  public: SearchModel *searchSimModel;
};

/////////////////////////////////////////////////
Palette::Palette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PalettePrivate)
{
  // The tab bar along the top.
  auto tabBar = new QTabBar;
  tabBar->setObjectName("plottingTabBar");
  tabBar->addTab("TOPICS");
  tabBar->addTab("SIM");
  tabBar->addTab("SEARCH");
  tabBar->setExpanding(true);
  tabBar->setDrawBase(false);
  tabBar->setFocusPolicy(Qt::NoFocus);

  // Create a view delegate, to handle drawing items in the tree view
  auto plotItemDelegate = new PlotItemDelegate;

  // The model that will hold data to be displayed in the topic tree view
  this->dataPtr->topicsModel = new PlotItemModel;
  this->FillTopics(this->dataPtr->topicsModel);

  // A proxy model to filter topic model
  this->dataPtr->searchTopicsModel = new SearchModel;
  this->dataPtr->searchTopicsModel->setFilterRole(
      PlotItemDelegate::TOPIC_NAME_ROLE);
  this->dataPtr->searchTopicsModel->setSourceModel(this->dataPtr->topicsModel);

  // A tree to visualize the topics and their messages.
  auto topicsTree = new QTreeView;
  topicsTree->setObjectName("plotTree");
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

  // A proxy model to filter sim model
  this->dataPtr->searchSimModel = new SearchModel;
  this->dataPtr->searchSimModel->setFilterRole(
      PlotItemDelegate::TOPIC_NAME_ROLE);
  this->dataPtr->searchSimModel->setSourceModel(this->dataPtr->simModel);

  // A tree to visualize sim variables
  auto simTree = new QTreeView;
  simTree->setObjectName("plotTree");
  simTree->setAnimated(true);
  simTree->setHeaderHidden(true);
  simTree->setExpandsOnDoubleClick(true);
  simTree->setModel(this->dataPtr->simModel);
  simTree->setItemDelegate(plotItemDelegate);
  simTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  simTree->setDragEnabled(true);
  simTree->setDragDropMode(QAbstractItemView::DragOnly);

  // Search field
  auto searchIcon = new QLabel();
  searchIcon->setPixmap(QPixmap(":/images/search.svg"));

  auto searchEdit = new QLineEdit();
  searchEdit->setObjectName("plotLineEdit");
  this->connect(searchEdit, SIGNAL(textChanged(QString)), this,
      SLOT(UpdateSearch(QString)));
  this->UpdateSearch("");

  auto searchField = new QHBoxLayout();
  searchField->addWidget(searchIcon);
  searchField->addWidget(searchEdit);

  // A tree to visualize topics search results
  auto searchTopicsTree = new QTreeView;
  searchTopicsTree->setObjectName("plotTree");
  searchTopicsTree->setAnimated(true);
  searchTopicsTree->setHeaderHidden(true);
  searchTopicsTree->setExpandsOnDoubleClick(true);
  searchTopicsTree->setModel(this->dataPtr->searchTopicsModel);
  searchTopicsTree->setItemDelegate(plotItemDelegate);
  searchTopicsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  searchTopicsTree->setDragEnabled(true);
  searchTopicsTree->setDragDropMode(QAbstractItemView::DragOnly);
  // searchTopicsTree->expandAll();

  // A tree to visualize sim search results
  auto searchSimTree = new QTreeView;
  searchSimTree->setObjectName("plotTree");
  searchSimTree->setAnimated(true);
  searchSimTree->setHeaderHidden(true);
  searchSimTree->setExpandsOnDoubleClick(true);
  searchSimTree->setModel(this->dataPtr->searchSimModel);
  searchSimTree->setItemDelegate(plotItemDelegate);
  searchSimTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  searchSimTree->setDragEnabled(true);
  searchSimTree->setDragDropMode(QAbstractItemView::DragOnly);
  // searchSimTree->expandAll();

  // Search layout
  auto topicsLabel = new QLabel(tr("TOPICS"));
  topicsLabel->setObjectName("plottingSearchLabel");
  auto simLabel = new QLabel(tr("SIM"));
  simLabel->setObjectName("plottingSearchLabel");

  auto searchLayout = new QVBoxLayout();
  searchLayout->addLayout(searchField);
  searchLayout->addWidget(topicsLabel);
  searchLayout->addWidget(searchTopicsTree);
  searchLayout->addWidget(simLabel);
  searchLayout->addWidget(searchSimTree);

  auto searchWidget = new QWidget();
  searchWidget->setLayout(searchLayout);

  // The stacked layout is used by the TabBar to switch active layouts
  auto tabStackedLayout = new QStackedLayout;
  tabStackedLayout->setContentsMargins(0, 0, 0, 0);
  tabStackedLayout->addWidget(topicsTree);
  tabStackedLayout->addWidget(simTree);
  tabStackedLayout->addWidget(searchWidget);

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
    std::string prefix = "/gazebo/default";
    auto shortName = topic;
    auto idX = shortName.find(prefix);
    if (idX != std::string::npos)
      shortName.replace(0, prefix.size(), "~");

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

  //=================
  // TODO for testing - remove later
  auto itema = new QStandardItem();
  itema->setData("Dog", PlotItemDelegate::TOPIC_NAME_ROLE);
  itema->setData("Dog", PlotItemDelegate::DATA_ROLE);
  itema->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
  _simModel->appendRow(itema);
  auto itemb = new QStandardItem("Cat");
  itemb->setData("Cat", PlotItemDelegate::TOPIC_NAME_ROLE);
  itemb->setData("Cat", PlotItemDelegate::DATA_ROLE);
  itemb->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
  _simModel->appendRow(itemb);
  auto itemc = new QStandardItem("Turtle");
  itemc->setData("Turtle", PlotItemDelegate::TOPIC_NAME_ROLE);
  itemc->setData("Turtle", PlotItemDelegate::DATA_ROLE);
  itemc->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
  _simModel->appendRow(itemc);
  auto simTimeItem = new QStandardItem("sim_time");
  simTimeItem->setData("sim_time", PlotItemDelegate::TOPIC_NAME_ROLE);
  simTimeItem->setData("sim_time", PlotItemDelegate::DATA_ROLE);
  simTimeItem->setData("Double", PlotItemDelegate::DATA_TYPE_NAME);
  _simModel->appendRow(simTimeItem);
  //=================
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

        auto *childItem = new QStandardItem();
        childItem->setData(humanName.c_str(),
            PlotItemDelegate::TOPIC_NAME_ROLE);

        switch (field->type())
        {
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
          default:
            continue;
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
            childItem->setData(QString::fromStdString(humanName),
                PlotItemDelegate::TOPIC_NAME_ROLE);
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

/////////////////////////////////////////////////
void Palette::UpdateSearch(const QString &_search)
{
  this->dataPtr->searchTopicsModel->setSearch(_search);
  this->dataPtr->searchSimModel->setSearch(_search);
}

