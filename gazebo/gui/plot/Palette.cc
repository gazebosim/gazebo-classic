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
class TopicsViewDelegate : public QStyledItemDelegate
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
    DATA_TYPE_NAME,
  };

  /// \brief Constructor
  public: TopicsViewDelegate() = default;

  /// \brief Destructor
  public: virtual ~TopicsViewDelegate() = default;

  /// \brief Custom paint function
  /// \param[in] _painter Pointer to the QT painter
  /// \param[in] _opt Item options
  /// \param[in] _index Item model index
  public: void paint(QPainter *_painter, const QStyleOptionViewItem &_opt,
                     const QModelIndex &_index) const
  {
    // Bounding box for each text line
    QRectF r = _opt.rect;

    // Custom options
    QString topicName = qvariant_cast<QString>(_index.data(TOPIC_NAME_ROLE));
    QString typeName = qvariant_cast<QString>(_index.data(DATA_TYPE_NAME));

    // Choose font
    QFont font;
    auto fontWeight = QFont::Normal;
    if (typeName == "title")
    {
      font.setFamily("Roboto Bold");
      fontWeight = QFont::Bold;

      // Erase the branch image for titles
      QRectF titleRect = _opt.rect;
      titleRect.setLeft(titleRect.left() - 13);
       QBrush brush(QColor("#dddddd"));
      _painter->save();
      _painter->fillRect(titleRect, brush);
      _painter->restore();
    }
    else
    {
      font.setFamily("Roboto Regular");
    }
    QFontMetrics fm(font);

    // Handle hover style
    if (_opt.state & QStyle::State_MouseOver)
    {
      _painter->setPen(QPen(QColor(200, 200, 200, 0), 0));
      _painter->setBrush(QColor(200, 200, 200));
      _painter->drawRect(_opt.rect);
    }

    // Paint the icon, if present
    if (typeName == "model" || typeName == "link" || typeName == "collision" ||
        typeName == "visual" || typeName == "joint")
    {
      double iconSize = 15;

      r.adjust(iconSize + 8, 5, 0, -5);
      QRectF iconRect = _opt.rect;
      iconRect.setTop(iconRect.top() + (_opt.rect.height()/2.0 - iconSize/2.0));

      // Paint icon
      _painter->setPen(QColor(90, 90, 90));
      _painter->setBrush(QColor(90, 90, 90));

      QIcon icon(":/images/" + typeName  + ".svg");
      _painter->drawPixmap(iconRect.left(), iconRect.top(),
          icon.pixmap(iconSize, iconSize));
    }
    else if (typeName == "title")
    {
      r.adjust(-15, 5, 0, -5);
    }
    // Paint the icon and data type, if present
    else if (!typeName.isEmpty())
    {
      double iconSize = 20;

      r.adjust(iconSize + 8, 3, 0, 0);

      QRectF iconRect = _opt.rect;
      iconRect.setTop(iconRect.top() + (_opt.rect.height()/2.0 - iconSize/2.0));

      // Paint icon
      _painter->setPen(QColor(90, 90, 90));
      _painter->setBrush(QColor(90, 90, 90));
      QIcon icon(":/images/graph_line.svg");
      _painter->drawPixmap(iconRect.left(), iconRect.top(),
          icon.pixmap(iconSize, iconSize));
    }
    else
    {
      // Otherwise use a rectangle that is sized for the just the topic name
      r.adjust(0, 5, 0, -5);
    }

    _painter->setFont(QFont(font.family(), font.pointSize(), fontWeight));
    _painter->setPen(QColor(30, 30, 30));
    _painter->drawText(r, topicName);
  }

  /// \brief Size hint tells QT how big an item is
  /// \param[in] _option Style options
  /// \param[in] _index Item model index
  public: QSize sizeHint(const QStyleOptionViewItem &_option,
                         const QModelIndex &_index) const
  {
    QSize size = QStyledItemDelegate::sizeHint(_option, _index);

    QFont font("Roboto Regular");
    QFontMetrics fm(font);

    size.setHeight(5 + fm.height() + 5);

    return size;
  }
};

/////////////////////////////////////////////////
/// Customize the item model so that we can pass along the correct MIME
/// information during a drag-drop.
class TopicsItemModel : public QStandardItemModel
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
            TopicsViewDelegate::DATA_ROLE).toString();
        curMimeData->setData("application/x-item", text.toLatin1().data());

        // Todo: add x-item.list functionality:
        // http://doc.qt.io/qt-4.8/model-view-programming.html#
        // using-drag-and-drop-with-item-views
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
  /// \brief Customize so we accept rows:
  /// * That match themselves, or
  /// * That have a parent that matches (on its own), or
  /// * That have a child that matches.
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if row matches at least one of the conditions.
  public: bool filterAcceptsRow(int _srcRow, const QModelIndex &_srcParent)
      const
  {
    if (this->filterAcceptsRowItself(_srcRow, _srcParent))
      return true;

    // accept if any of the parents is accepted on it's own merits
    QModelIndex parentIndex = _srcParent;
    while (parentIndex.isValid())
    {
      if (this->filterAcceptsRowItself(parentIndex.row(), parentIndex.parent()))
        return true;
      parentIndex = parentIndex.parent();
    }

    // accept if any of the children is accepted on it's own merits
    if (this->hasAcceptedChildren(_srcRow, _srcParent))
    {
      return true;
    }

    return false;
  }

  /// \brief Check if the original filter accepts this row.
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if row matches at least one of the conditions.
  public: bool filterAcceptsRowItself(const int _srcRow, const
      QModelIndex &_srcParent) const
  {
    return QSortFilterProxyModel::filterAcceptsRow(_srcRow, _srcParent);
  }

  /// \brief Check if any of the children match the filter.
  /// \param[in] _srcRow Row on the source model.
  /// \param[in] _srcParent Parent on the source model.
  /// \return True if row matches at least one of the conditions.
  public: bool hasAcceptedChildren(const int _srcRow,
      const QModelIndex &_srcParent) const
  {
    QModelIndex item = sourceModel()->index(_srcRow, 0, _srcParent);
    if (!item.isValid())
      return false;

    int childCount = item.model()->rowCount(item);
    if (childCount == 0)
      return false;

    for (int i = 0; i < childCount; ++i)
    {
      if (this->filterAcceptsRowItself(i, item))
        return true;

      if (this->hasAcceptedChildren(i, item))
        return true;
    }

    return false;
  }
};

/////////////////////////////////////////////////
/// \brief Private data for the Palette class
class gazebo::gui::PalettePrivate
{
  /// \brief Top pane of the topics tab.
  public: QFrame *mainFrame;

  public: TopicsItemModel *topicsModel;
  public: TopicsItemModel *modelsModel;
  public: TopicsItemModel *simModel;
  public: SearchModel *searchTopicsModel;
  public: SearchModel *searchModelsModel;
  public: SearchModel *searchSimModel;
};

/////////////////////////////////////////////////
Palette::Palette(QWidget *_parent) : QWidget(_parent),
    dataPtr(new PalettePrivate)
{
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Regular.ttf");
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Bold.ttf");
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Black.ttf");
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Medium.ttf");
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Light.ttf");
  QFontDatabase::addApplicationFont(":/fonts/Roboto-Thin.ttf");
//  QFont newFont(":/fonts/Roboto-Regular.ttf");
  //QApplication::setFont(newFont);

  // The tab bar along the top.
  auto tabBar = new QTabBar;
  tabBar->addTab("Topics");
  tabBar->addTab("Models");
  tabBar->addTab("Sim");
  tabBar->addTab("Search");
  tabBar->setExpanding(true);
  tabBar->setDrawBase(false);
  tabBar->setFocusPolicy(Qt::NoFocus);

  // Create a view delegate, to handle drawing items in the topicTree view
  TopicsViewDelegate *topicsViewDelegate = new TopicsViewDelegate;

  // The model that will hold data to be displayed in the topicTree view
  this->dataPtr->topicsModel = new TopicsItemModel;
  this->FillTopics(this->dataPtr->topicsModel);

  // A proxy model to filter topic model
  this->dataPtr->searchTopicsModel = new SearchModel;
  this->dataPtr->searchTopicsModel->setFilterRole(
      TopicsViewDelegate::TOPIC_NAME_ROLE);
  this->dataPtr->searchTopicsModel->setSourceModel(this->dataPtr->topicsModel);
  this->dataPtr->searchTopicsModel->setFilterCaseSensitivity(
      Qt::CaseInsensitive);

  // A tree to visualize the topics and their messages.
  QTreeView *topicsTree = new QTreeView;
  topicsTree->setObjectName("topicList");
  topicsTree->setAnimated(true);
  topicsTree->setHeaderHidden(true);
  topicsTree->setExpandsOnDoubleClick(true);
  topicsTree->setModel(this->dataPtr->topicsModel);
  topicsTree->setItemDelegate(topicsViewDelegate);
  topicsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  topicsTree->setDragEnabled(true);
  topicsTree->setDragDropMode(QAbstractItemView::DragOnly);

  // The model that will hold data to be displayed in the modelTree view
  this->dataPtr->modelsModel = new TopicsItemModel;
  this->FillModels(this->dataPtr->modelsModel);

  // A proxy model to filter model model
  this->dataPtr->searchModelsModel = new SearchModel;
  this->dataPtr->searchModelsModel->setFilterRole(
      TopicsViewDelegate::TOPIC_NAME_ROLE);
  this->dataPtr->searchModelsModel->setSourceModel(this->dataPtr->modelsModel);
  this->dataPtr->searchModelsModel->setFilterCaseSensitivity(
      Qt::CaseInsensitive);

  // A tree to visualize the models and their messages.
  QTreeView *modelsTree = new QTreeView;
  modelsTree->setObjectName("topicList");
  modelsTree->setAnimated(true);
  modelsTree->setHeaderHidden(true);
  modelsTree->setExpandsOnDoubleClick(true);
  modelsTree->setModel(this->dataPtr->modelsModel);
  modelsTree->setItemDelegate(topicsViewDelegate);
  modelsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  modelsTree->setDragEnabled(true);
  modelsTree->setDragDropMode(QAbstractItemView::DragOnly);

  // The model that will hold data to be displayed in the simTree view
  this->dataPtr->simModel = new TopicsItemModel;
  this->FillSim(this->dataPtr->simModel);

  // A proxy model to filter sim model
  this->dataPtr->searchSimModel = new SearchModel;
  this->dataPtr->searchSimModel->setFilterRole(
      TopicsViewDelegate::TOPIC_NAME_ROLE);
  this->dataPtr->searchSimModel->setSourceModel(this->dataPtr->simModel);
  this->dataPtr->searchSimModel->setFilterCaseSensitivity(
      Qt::CaseInsensitive);

  // A tree to visualize sim variables
  QTreeView *simTree = new QTreeView;
  simTree->setObjectName("topicList");
  simTree->setAnimated(true);
  simTree->setHeaderHidden(true);
  simTree->setExpandsOnDoubleClick(true);
  simTree->setModel(this->dataPtr->simModel);
  simTree->setItemDelegate(topicsViewDelegate);
  simTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  simTree->setDragEnabled(true);
  simTree->setDragDropMode(QAbstractItemView::DragOnly);

  // Search line edit
  auto searchEdit = new QLineEdit();
  this->connect(searchEdit, SIGNAL(textChanged(QString)), this,
      SLOT(UpdateSearch(QString)));
  this->UpdateSearch("");

  // TODO Complete with other models too
  auto completer = new QCompleter(this->dataPtr->topicsModel, this);
  completer->setCaseSensitivity(Qt::CaseInsensitive);
  searchEdit->setCompleter(completer);

  // A tree to visualize topics search results
  QTreeView *searchTopicsTree = new QTreeView;
  searchTopicsTree->setObjectName("topicList");
  searchTopicsTree->setAnimated(true);
  searchTopicsTree->setHeaderHidden(true);
  searchTopicsTree->setExpandsOnDoubleClick(true);
  searchTopicsTree->setModel(this->dataPtr->searchTopicsModel);
  searchTopicsTree->setItemDelegate(topicsViewDelegate);
  searchTopicsTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  searchTopicsTree->setDragEnabled(true);
  searchTopicsTree->setDragDropMode(QAbstractItemView::DragOnly);

  // A tree to visualize sim search results
  QTreeView *searchSimTree = new QTreeView;
  searchSimTree->setObjectName("topicList");
  searchSimTree->setAnimated(true);
  searchSimTree->setHeaderHidden(true);
  searchSimTree->setExpandsOnDoubleClick(true);
  searchSimTree->setModel(this->dataPtr->searchSimModel);
  searchSimTree->setItemDelegate(topicsViewDelegate);
  searchSimTree->setEditTriggers(QAbstractItemView::NoEditTriggers);
  searchSimTree->setDragEnabled(true);
  searchSimTree->setDragDropMode(QAbstractItemView::DragOnly);

  // Search layout
  auto topicsLabel = new QLabel(tr("Topics"));
  topicsLabel->setObjectName("plottingSearchLabel");
  auto simLabel = new QLabel(tr("Sim"));
  simLabel->setObjectName("plottingSearchLabel");

  auto searchLayout = new QVBoxLayout();
  searchLayout->addWidget(searchEdit);
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
  tabStackedLayout->addWidget(modelsTree);
  tabStackedLayout->addWidget(simTree);
  tabStackedLayout->addWidget(searchWidget);

  // Connect TabBar to StackedLayout
  connect(tabBar, SIGNAL(currentChanged(int)),
          tabStackedLayout, SLOT(setCurrentIndex(int)));

  auto mainFrameLayout = new QVBoxLayout;
  mainFrameLayout->addWidget(tabBar);
  mainFrameLayout->addLayout(tabStackedLayout);
  mainFrameLayout->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->mainFrame = new QFrame(this);
  this->dataPtr->mainFrame->setObjectName("plotPaletteFrame");
  this->dataPtr->mainFrame->setLayout(mainFrameLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(this->dataPtr->mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);

  // Just a minimum size to prevent tab bar scrolling
  this->setMinimumWidth(320);

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
  QList<QStandardItem *> items;

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

    QStandardItem *topicItem = new QStandardItem();
    topicItem->setData(shortName.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
    _topicsModel->appendRow(topicItem);

    // Create a message from this topic
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
void Palette::FillModels(QStandardItemModel *_modelsModel)
{
  // Hard-coded values
  std::vector<std::string> models = {"robot_model", "another_model",
      "one_more_model_here"};
  std::vector<std::string> nestedModels = {"a_model_inside_a_model",
      "nested_model_ftw"};
  std::vector<std::string> links = {"a_link", "another_link",
      "lots_and_lots_of_links_"};
  std::vector<std::string> joints = {"I_rotate", "I_am_fixed",
      "i_do_complex_things"};
  std::vector<std::string> collisions = {"collision_1", "that_other_collision"};
  std::vector<std::string> visuals = {"visual_1", "that_other_visual"};
  std::vector<std::string> properties = {"pose", "velocity", "acceleration"};
  std::vector<std::string> orientations = {"roll", "pitch", "yaw"};

  if (!models.empty())
  {
    auto modelsTitle = new QStandardItem();
    modelsTitle->setData("Models", TopicsViewDelegate::TOPIC_NAME_ROLE);
    modelsTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
    _modelsModel->appendRow(modelsTitle);
  }

  for (auto model : models)
  {
    auto modelItem = new QStandardItem();
    modelItem->setData(model.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
    modelItem->setData("model", TopicsViewDelegate::DATA_TYPE_NAME);
    _modelsModel->appendRow(modelItem);

    if (!nestedModels.empty())
    {
      auto nestedModelsTitle = new QStandardItem();
      nestedModelsTitle->setData("Models", TopicsViewDelegate::TOPIC_NAME_ROLE);
      nestedModelsTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(nestedModelsTitle);
    }

    for (auto nestedModel : nestedModels)
    {
      auto nestedModelItem = new QStandardItem();
      nestedModelItem->setData(nestedModel.c_str(),
          TopicsViewDelegate::TOPIC_NAME_ROLE);
      nestedModelItem->setData("model", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(nestedModelItem);
    }

    if (!links.empty())
    {
      auto linksTitle = new QStandardItem();
      linksTitle->setData("Links", TopicsViewDelegate::TOPIC_NAME_ROLE);
      linksTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(linksTitle);
    }

    for (auto link : links)
    {
      auto linkItem = new QStandardItem();
      linkItem->setData(link.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
      linkItem->setData("link", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(linkItem);

      if (!collisions.empty())
      {
        auto collisionsTitle = new QStandardItem();
        collisionsTitle->setData("Collisions",
            TopicsViewDelegate::TOPIC_NAME_ROLE);
        collisionsTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
        linkItem->appendRow(collisionsTitle);
      }

      for (auto collision : collisions)
      {
        auto collisionItem = new QStandardItem();
        collisionItem->setData(collision.c_str(),
            TopicsViewDelegate::TOPIC_NAME_ROLE);
        collisionItem->setData("collision", TopicsViewDelegate::DATA_TYPE_NAME);
        linkItem->appendRow(collisionItem);

        for (auto properti : properties)
        {
          auto propertiItem = new QStandardItem();
          propertiItem->setData(properti.c_str(),
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          collisionItem->appendRow(propertiItem);

          auto positionItem = new QStandardItem();
          positionItem->setData("position",
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          propertiItem->appendRow(positionItem);

          auto orientationItem = new QStandardItem();
          orientationItem->setData("orientation",
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          propertiItem->appendRow(orientationItem);

          for (auto orientation : orientations)
          {
            auto oriItem = new QStandardItem();
            oriItem->setData(orientation.c_str(),
                TopicsViewDelegate::TOPIC_NAME_ROLE);
            orientationItem->appendRow(oriItem);

            oriItem->setToolTip(tr(
                  "<font size=3><p><b>Type</b>: Double</p></font>"));
            oriItem->setDragEnabled(true);
            oriItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);

            std::string dataName = model + "/" + link + "/" + collision + "/" +
                properti + "/orientation/" + orientation;
            oriItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
          }
        }
      }

      if (!visuals.empty())
      {
        auto visualsTitle = new QStandardItem();
        visualsTitle->setData("Visuals",
            TopicsViewDelegate::TOPIC_NAME_ROLE);
        visualsTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
        linkItem->appendRow(visualsTitle);
      }

      for (auto visual : visuals)
      {
        auto visualItem = new QStandardItem();
        visualItem->setData(visual.c_str(),
            TopicsViewDelegate::TOPIC_NAME_ROLE);
        visualItem->setData("visual", TopicsViewDelegate::DATA_TYPE_NAME);
        linkItem->appendRow(visualItem);

        for (auto properti : properties)
        {
          auto propertiItem = new QStandardItem();
          propertiItem->setData(properti.c_str(),
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          visualItem->appendRow(propertiItem);

          auto positionItem = new QStandardItem();
          positionItem->setData("position",
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          propertiItem->appendRow(positionItem);

          auto orientationItem = new QStandardItem();
          orientationItem->setData("orientation",
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          propertiItem->appendRow(orientationItem);

          for (auto orientation : orientations)
          {
            auto oriItem = new QStandardItem();
            oriItem->setData(orientation.c_str(),
                TopicsViewDelegate::TOPIC_NAME_ROLE);
            orientationItem->appendRow(oriItem);

            oriItem->setToolTip(tr(
                  "<font size=3><p><b>Type</b>: Double</p></font>"));
            oriItem->setDragEnabled(true);
            oriItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);

            std::string dataName = model + "/" + link + "/" + visual + "/" +
                properti + "/orientation/" + orientation;
            oriItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
          }
        }
      }
    }

    if (!joints.empty())
    {
      auto jointsTitle = new QStandardItem();
      jointsTitle->setData("Joints", TopicsViewDelegate::TOPIC_NAME_ROLE);
      jointsTitle->setData("title", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(jointsTitle);
    }

    for (auto joint : joints)
    {
      auto jointItem = new QStandardItem();
      jointItem->setData(joint.c_str(),
          TopicsViewDelegate::TOPIC_NAME_ROLE);
      jointItem->setData("joint", TopicsViewDelegate::DATA_TYPE_NAME);
      modelItem->appendRow(jointItem);
    }
  }
}

/////////////////////////////////////////////////
void Palette::FillSim(QStandardItemModel *_simModel)
{
  // Hard-coded values
  std::multimap<std::string, std::string> simFields = {
      {"~/world_stats", "sim_time"},
      {"~/world_stats", "real_time"},
      {"~/world_stats", "iterations"}};

  for (auto field : simFields)
  {
    auto childItem = new QStandardItem();
    childItem->setDragEnabled(true);

    auto humanName = ConfigWidget::HumanReadableKey(field.second);
    childItem->setData(humanName.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);

    if (field.second == "iterations")
      childItem->setData("Uint 64", TopicsViewDelegate::DATA_TYPE_NAME);
    else
      childItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);

    std::string typeName =
        "<font size=3><p><b>Type</b>:" + childItem->data(
        TopicsViewDelegate::DATA_TYPE_NAME).toString().toStdString() +
        "</p></font>";
    childItem->setToolTip(QString::fromStdString(typeName));

    std::string dataName = field.first + "?p=/" + field.second;
    childItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);

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
        childItem->setToolTip(tr(
              "<font size=3><p>Drag to y label to plot.</p></font>"));
        childItem->setData(humanName.c_str(),
            TopicsViewDelegate::TOPIC_NAME_ROLE);

        switch (field->type())
        {
          default:
          case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
            childItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_FLOAT:
            childItem->setData("Float", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_INT64:
            childItem->setData("Int 64", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_UINT64:
            childItem->setData("Uint 64", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_INT32:
            childItem->setData("Int 32", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_UINT32:
            childItem->setData("Uint 32", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
          case google::protobuf::FieldDescriptor::TYPE_BOOL:
            childItem->setData("Bool", TopicsViewDelegate::DATA_TYPE_NAME);
            break;
        }

        std::string dataName = _uri + "/" + name;
        childItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
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
          childItem->setToolTip(tr(
              "<font size=3><p>Drag to y label to plot.</p></font>"));
          childItem->setData(humanName.c_str(),
              TopicsViewDelegate::TOPIC_NAME_ROLE);
          childItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
          childItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);
          childItem->setDragEnabled(true);

          _item->appendRow(childItem);
        }
        // Custom XYZ widget for position
        else if (field->message_type()->name() == "Vector3d")
        {
          auto *posItem = new QStandardItem();
          posItem->setData(name.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
          _item->appendRow(posItem);

          std::vector<std::string> xyz = {"x", "y", "z"};
          for (auto it : xyz)
          {
            auto humanName = ConfigWidget::HumanReadableKey(it);
            std::string dataName = _uri + "/" + name + "/" + it;

            auto *childItem = new QStandardItem();
            childItem->setData(it.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
            childItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
            childItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);

            childItem->setToolTip(tr(
                "<font size=3><p>Drag to y label to plot.</p></font>"));
            childItem->setDragEnabled(true);

            posItem->appendRow(childItem);
          }
        }
        // Custom RPY widgets for orientation
        else if (field->message_type()->name() == "Quaternion")
        {
          auto *quatItem = new QStandardItem();
          quatItem->setData(name.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
          _item->appendRow(quatItem);

          std::vector<std::string> rpy = {"roll", "pitch", "yaw"};
          for (auto it : rpy)
          {
            auto humanName = ConfigWidget::HumanReadableKey(it);
            std::string dataName = _uri + "/" + name + "/" + it;
            auto *childItem = new QStandardItem();
            childItem->setData(it.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
            childItem->setData(dataName.c_str(), TopicsViewDelegate::DATA_ROLE);
            childItem->setData("Double", TopicsViewDelegate::DATA_TYPE_NAME);

            childItem->setToolTip(tr(
                "<font size=3><p>Drag to y label to plot.</p></font>"));
            childItem->setDragEnabled(true);

            quatItem->appendRow(childItem);
          }
        }
        // Create a collapsible list for submessages
        else
        {
          auto fieldMsg = (ref->MutableMessage(_msg, field));
          auto *childItem = new QStandardItem();
          childItem->setData(name.c_str(), TopicsViewDelegate::TOPIC_NAME_ROLE);
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

  if (exp == "")
    exp = "_no_match_";

  this->dataPtr->searchTopicsModel->setFilterRegExp(exp);
  this->dataPtr->searchSimModel->setFilterRegExp(exp);
}
