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

#include "gazebo/common/SVGLoader.hh"
#include "gazebo/gui/plot/ExportDialog.hh"

using namespace gazebo;
using namespace gui;

class PlotViewDelegate : public QStyledItemDelegate
{
  public: enum datarole {headerTextRole = Qt::UserRole + 100,subHeaderTextrole = Qt::UserRole+101,IconRole = Qt::UserRole+102};

  public: PlotViewDelegate()
          {
          }

  public: virtual ~PlotViewDelegate()
          {
          }

  public: void paint(QPainter *_painter, const QStyleOptionViewItem &_opt,
              const QModelIndex &_index) const
          {
            QRectF r = _opt.rect;

            _painter->save();

            // Add margins to the rectangle
            r.adjust(5, 5, -5, -5);
            _painter->setBrush(QColor(100, 100, 100));
            _painter->drawRect(r);

            _painter->drawText(r, Qt::AlignCenter | Qt::AlignBottom, "aa");
            _painter->restore();
          }

  public: QSize sizeHint(const QStyleOptionViewItem & /*_option*/,
                              const QModelIndex & /*_index*/) const
          {
            QFont font = QApplication::font();
            QFontMetrics fm(font);
            return QSize(120, 80);
          }
};

/////////////////////////////////////////////////
ExportDialog::ExportDialog(QWidget *_parent)
: QDialog(_parent)
{
  QHBoxLayout *titleLayout = new QHBoxLayout;
  titleLayout->addWidget(new QLabel("Export Plot"));
  titleLayout->setAlignment(Qt::AlignHCenter);

  this->setObjectName("PlotExportDialog");
  this->setWindowTitle("Export Plot");
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                       Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *exportButton = new QPushButton("&Export");
  exportButton->setDefault(true);
  connect(exportButton, SIGNAL(clicked()), this, SLOT(OnExport()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(exportButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  //QVBoxLayout *plotsLayout = new QVBoxLayout;
  QListView *listView = new QListView;

  QStandardItemModel *model = new QStandardItemModel();

  for (int yp = 0; yp < 4; ++yp)
  {
    for (int xp = 0; xp < 2; ++xp)
    {
      QStandardItem *item = new QStandardItem();
      item->setData("Test", PlotViewDelegate::headerTextRole);
      item->setEditable(false);
      model->appendRow(item);

      //QAction *action = new QAction(tr("test"), this);
      //listView->addAction(action);
    }
  }

  //QAbstractItemModel *model = new QStringListModel(test);
  listView->setViewMode(QListView::IconMode);
  listView->setWrapping(true);
  listView->setFlow(QListView::LeftToRight);
  listView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  listView->setResizeMode(QListView::Adjust);
  listView->setMovement(QListView::Static);

  PlotViewDelegate *plotViewDelegate = new PlotViewDelegate;

  listView->setModel(model);
  listView->setItemDelegate(plotViewDelegate);

  /*QScrollArea *plotScroll = new QScrollArea;
  QFrame *plotsFrame = new QFrame;

  QGridLayout *plotGridLayout = new QGridLayout;
  for (int yp = 0; yp < 4; ++yp)
  {
    for (int xp = 0; xp < 2; ++xp)
    {

      QFrame *plotFrame = new QFrame;
      plotFrame->setStyleSheet("QFrame { background-color: #dedede; padding: 10px; margin:4px; border: 0px}");

      QVBoxLayout *plotFrameLayout = new QVBoxLayout;
      plotFrameLayout->addWidget(new QPushButton("test"));
      plotFrame->setLayout(plotFrameLayout);
      QGraphicsDropShadowEffect *effect = new QGraphicsDropShadowEffect();
      effect->setBlurRadius(4);
      effect->setOffset(2, 2);
      plotFrame->setGraphicsEffect(effect);

      plotGridLayout->addWidget(plotFrame, yp, xp);
    }
  }

  plotsFrame->setLayout(plotGridLayout);
  plotScroll->setWidget(plotsFrame);
  */
  //plotsLayout->addWidget(listView);//plotScroll);
  /*QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(this->pathLineEdit, 0, 0);
  gridLayout->addWidget(browseButton, 0, 1);
  gridLayout->addWidget(nameLabel, 1, 0);
  gridLayout->addWidget(nameLineEdit, 1, 1);
  */

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(titleLayout);
  mainLayout->addWidget(listView);
  //mainLayout->addWidget(this->messageLabel);
  //mainLayout->addLayout(gridLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  // Set a reasonable default size.
  this->resize(640, 400);
  //this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
ExportDialog::~ExportDialog()
{
}

/////////////////////////////////////////////////
void ExportDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void ExportDialog::OnExport()
{
}

/////////////////////////////////////////////////
void ExportDialog::showEvent(QShowEvent */*_event*/)
{
}
