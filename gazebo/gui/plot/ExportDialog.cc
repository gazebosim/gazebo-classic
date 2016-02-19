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
#include <fstream>
#include <functional>

#include "gazebo/common/SVGLoader.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotWindow.hh"
#include "gazebo/gui/plot/ExportDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
class gazebo::gui::ExportDialogPrivate
{
  public: QListView *listView;
  public: QPushButton *exportButton;
};

/////////////////////////////////////////////////
// Subclass QStandardItem so that we can store a pointer to the plot canvas
class PlotViewItem : public QStandardItem
{
  public: PlotCanvas *canvas;
};

/////////////////////////////////////////////////
class PlotViewDelegate : public QStyledItemDelegate
{
  public: enum datarole
          {
            headerTextRole = Qt::UserRole + 100,
            iconRole = Qt::UserRole+101
          };

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
            QRectF iconRect = _opt.rect;
            QIcon icon = qvariant_cast<QIcon>(_index.data(iconRole));
            QString title = qvariant_cast<QString>(_index.data(headerTextRole));

            QFont font = QApplication::font();
            QFontMetrics fm(font);

            _painter->save();

            // Add margins to the rectangle
            r.adjust(5, 5, -5, -5);
            /*if (_opt.state & QStyle::State_Selected)
            {
              _painter->setBrush(QColor(200, 200, 200));
              _painter->setPen(QColor(255, 255, 255));
            }
            else
            {
              _painter->setBrush(QColor(90, 90, 90));
              _painter->setPen(QColor(0, 0, 0));
            }*/
            //_painter->drawRect(r);

            iconRect.adjust(10, 10, -10, -10);

            QPixmap image = icon.pixmap(iconRect.width(), iconRect.height());
            _painter->drawPixmap(iconRect.left(), iconRect.top(), image);

            _painter->setPen(QColor(0, 0, 0));

            QPixmap checkImage;

            if (_opt.state & QStyle::State_Selected)
             checkImage.load(":/images/check_box_black.svg");
            else
             checkImage.load(":/images/check_box_outline_black.svg");

            int checkSize = 24;
            int checkMargin = 4;
            int checkTitleWidth = fm.width(title) + checkSize + checkMargin;

            QRectF checkRect = _opt.rect;
            checkRect.setTop(iconRect.top() + image.height() + checkMargin);
            checkRect.setLeft(iconRect.left() +
                (iconRect.width() - checkTitleWidth)/2.0);
            checkRect.setWidth(checkSize);
            checkRect.setHeight(checkSize);
            _painter->drawPixmap(checkRect, checkImage, checkImage.rect());

            QRectF titleRect = _opt.rect;
            titleRect.setTop(checkRect.top() +
                (checkRect.height() - fm.height())/2.0);
            titleRect.setLeft(
                checkRect.left() + checkRect.width() + checkMargin);
            _painter->drawText(titleRect, Qt::AlignVCenter, title);
            _painter->restore();
          }

  public: QSize sizeHint(const QStyleOptionViewItem &_option,
                              const QModelIndex &_index) const
          {
            QIcon icon = qvariant_cast<QIcon>(_index.data(iconRole));
            QSize iconSize = icon.actualSize(_option.decorationSize);

            iconSize.scale(320, 180, Qt::KeepAspectRatio);

            QFont font = QApplication::font();
            QFontMetrics fm(font);
            QSize result = QSize(iconSize.width(),
                iconSize.height() + fm.height() + 10);
            return result;
          }
};

/////////////////////////////////////////////////
ExportDialog::ExportDialog(QWidget *_parent)
: QDialog(_parent),
  dataPtr(new ExportDialogPrivate)
{
  QAction *selectAllAct = new QAction(
      QIcon(":/images/select_all.svg"), tr("Select all"), this);
  selectAllAct->setToolTip(tr("Select all"));
  selectAllAct->setVisible(true);

  QAction *clearAct = new QAction(
      QIcon(":/images/clear.svg"), tr("Clear selection"), this);
  clearAct->setToolTip(tr("Clear selection"));
  clearAct->setVisible(true);

  QToolBar *selectToolbar = new QToolBar;
  selectToolbar->setObjectName("plotToolbar");
  selectToolbar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  selectToolbar->addAction(selectAllAct);
  selectToolbar->addAction(clearAct);

  QHBoxLayout *titleLayout = new QHBoxLayout;
  titleLayout->addWidget(new QLabel("Select plots to export", this));
  titleLayout->addStretch();
  titleLayout->addWidget(selectToolbar);
  titleLayout->setAlignment(Qt::AlignHCenter);
  titleLayout->setContentsMargins(0, 0, 0, 0);

  QFrame *titleFrame = new QFrame;
  titleFrame->setObjectName("plotExportTitleFrame");
  titleFrame->setLayout(titleLayout);

  this->setObjectName("plotExport");
  this->setWindowTitle("Export Plot");
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                       Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  cancelButton->setObjectName("flat");
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QMenu *exportMenu = new QMenu;
  exportMenu->addAction("CSV");
  exportMenu->addAction("PDF");
  exportMenu->addAction("PNG");

  this->dataPtr->exportButton = new QPushButton("&Export to");
  this->dataPtr->exportButton->setObjectName("flat");
  this->dataPtr->exportButton->setDefault(true);
  this->dataPtr->exportButton->setEnabled(false);
  this->dataPtr->exportButton->setMenu(exportMenu);

  /*connect(this->dataPtr->exportButton, SIGNAL(clicked()),
          this, SLOT(OnExport()));
          */
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addStretch(2);
  buttonsLayout->addWidget(this->dataPtr->exportButton);

  this->dataPtr->listView = new QListView;
  this->dataPtr->listView->setObjectName("plotExportListView");

  QStandardItemModel *model = new QStandardItemModel();

  PlotWindow *plotWindow = static_cast<PlotWindow*>(_parent);
  std::list<PlotCanvas*> plots = plotWindow->Plots();

  for (auto &plot : plots)
  {
    QIcon icon(QPixmap::grabWindow(plot->winId()));

    PlotViewItem *item = new PlotViewItem;
    item->canvas = plot;

    item->setData(plot->Title(), PlotViewDelegate::headerTextRole);
    item->setData(icon, PlotViewDelegate::iconRole);
    item->setEditable(false);
    item->setCheckable(true);
    model->appendRow(item);
  }

  this->dataPtr->listView->setViewMode(QListView::IconMode);
  this->dataPtr->listView->setWrapping(true);
  this->dataPtr->listView->setFlow(QListView::LeftToRight);
  this->dataPtr->listView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  this->dataPtr->listView->setResizeMode(QListView::Adjust);
  this->dataPtr->listView->setMovement(QListView::Static);
  this->dataPtr->listView->setSelectionMode(QAbstractItemView::MultiSelection);
  connect(this->dataPtr->listView, SIGNAL(clicked(const QModelIndex &)),
          this, SLOT(OnSelected()));

  PlotViewDelegate *plotViewDelegate = new PlotViewDelegate;

  this->dataPtr->listView->setModel(model);
  this->dataPtr->listView->setItemDelegate(plotViewDelegate);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(titleFrame);
  mainLayout->addWidget(this->dataPtr->listView);
  //mainLayout->addWidget(this->messageLabel);
  //mainLayout->addLayout(gridLayout);
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Set a reasonable default size.
  this->resize(640, 400);

  connect(clearAct, SIGNAL(triggered()),
          this->dataPtr->listView, SLOT(clearSelection()));
  connect(clearAct, SIGNAL(triggered()),
          this, SLOT(OnSelected()));

  connect(selectAllAct, SIGNAL(triggered()),
          this->dataPtr->listView, SLOT(selectAll()));
  connect(selectAllAct, SIGNAL(triggered()),
          this, SLOT(OnSelected()));
}

/////////////////////////////////////////////////
//void ExportDialog::OnSelected(const QModelIndex & /*_index*/)
void ExportDialog::OnSelected()
{
  this->dataPtr->exportButton->setEnabled(
      this->dataPtr->listView->selectionModel()->selectedIndexes().size() > 0);
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
  QFileDialog fileDialog(this, tr("Save Directory"), QDir::homePath());
  fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
  fileDialog.setAcceptMode(QFileDialog::AcceptSave);
  fileDialog.setFileMode(QFileDialog::DirectoryOnly);

  if (fileDialog.exec() == QDialog::Accepted)
  {
    QStringList selected = fileDialog.selectedFiles();

    if (selected.empty())
      return;

    std::string dir = selected[0].toStdString();

    QModelIndexList selectedPlots =
      this->dataPtr->listView->selectionModel()->selectedIndexes();
    for (auto iter = selectedPlots.begin(); iter != selectedPlots.end(); ++iter)
    {
      PlotViewItem *plotItem =
        static_cast<PlotViewItem*>(
            static_cast<QStandardItemModel*>(
              this->dataPtr->listView->model())->itemFromIndex(*iter));

      if (plotItem)
      {
        std::string title =
          plotItem->canvas->Title().toStdString();

        for (const auto &plot : plotItem->canvas->Plots())
        {
          for (const auto &curve : plot->Curves())
          {
            auto c = curve.lock();
            if (!c)
              continue;

            std::ofstream out(dir + "/" + title + "-" + c->Label() + ".csv");
            out << "x, " << c->Label() << std::endl;
            for (unsigned int j = 0; j < c->Size(); ++j)
            {
              ignition::math::Vector2d pt = c->Point(j);
              out << pt.X() << ", " << pt.Y() << std::endl;
            }
            out.close();
          }
        }
      }
      else
      {
        std::cout << "Error!!!\n";
      }
    }
    this->close();
  }
}
