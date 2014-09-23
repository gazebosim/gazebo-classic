/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportImageDialog::ImportImageDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->view = static_cast<EditorView*>(_parent);

  this->setWindowTitle(tr("Import Image"));

  this->fileLineEdit = new QLineEdit();
  this->fileLineEdit->setPlaceholderText(tr("Image file name"));
  connect(this, SIGNAL(SetFileName(QString)),
      this->fileLineEdit, SLOT(setText(QString)), Qt::QueuedConnection);

  QPushButton *fileButton = new QPushButton(tr("..."));
  connect(fileButton, SIGNAL(clicked()), this, SLOT(OnSelectFile()));

  QHBoxLayout *fileLayout = new QHBoxLayout;
  fileLayout->addWidget(new QLabel(tr("File: ")));
  fileLayout->addWidget(this->fileLineEdit);
  fileLayout->addWidget(fileButton);

  this->resolutionSpin = new QDoubleSpinBox;
  this->resolutionSpin->setRange(0.001, 1000);
  this->resolutionSpin->setSingleStep(0.01);
  this->resolutionSpin->setDecimals(4);
  this->resolutionSpin->setValue(0.1);

  QHBoxLayout *resolutionLayout = new QHBoxLayout;
  resolutionLayout->addWidget(new QLabel("Resolution (m/px):"));
  resolutionLayout->addStretch(1);
  resolutionLayout->addWidget(this->resolutionSpin);

  QDialogButtonBox *okCancelButtons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  connect(okCancelButtons, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancelButtons, SIGNAL(rejected()), this, SLOT(reject()));

  connect(this, SIGNAL(accepted()), this, SLOT(OnAccept()));

  this->imageDisplayWidth = 400;
  this->imageDisplayHeight = 300;
  QGraphicsScene *imageDisplayScene = new QGraphicsScene();
  imageDisplayScene->setSceneRect(0, 0, this->imageDisplayWidth,
                                        this->imageDisplayHeight);

  this->imageDisplay = new QGraphicsView(imageDisplayScene);
  this->imageDisplay->setBackgroundBrush(QBrush(Qt::white, Qt::SolidPattern));
  this->imageDisplay->setMouseTracking(true);
  this->imageDisplay->installEventFilter(this);

  QGraphicsTextItem *noImageText = new QGraphicsTextItem;
  noImageText->setPlainText("No image selected");
  noImageText->setDefaultTextColor(Qt::gray);
  this->imageDisplay->scene()->addItem(noImageText);

  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->addLayout(fileLayout, 0, 0);
  mainLayout->addLayout(resolutionLayout, 1, 0);
  mainLayout->addWidget(okCancelButtons, 2, 0);
  mainLayout->addWidget(this->imageDisplay, 0, 1, 4, 1);

  this->setLayout(mainLayout);

  this->drawingLine = false;
}

/////////////////////////////////////////////////
ImportImageDialog::~ImportImageDialog()
{
}

/////////////////////////////////////////////////
void ImportImageDialog::OnAccept()
{
  std::string filename = this->fileLineEdit->text().toStdString();
  if (!filename.empty())
  {
    this->view->SetBackgroundImage(filename, this->resolutionSpin->value());
  }
}

/////////////////////////////////////////////////
void ImportImageDialog::OnSelectFile()
{
  std::string filename = QFileDialog::getOpenFileName(this,
      tr("Open Image"), "",
      tr("Image Files (*.png *.jpg)")).toStdString();

  if (!filename.empty())
  {
    this->SetFileName(QString::fromStdString(filename));

    QList<QGraphicsItem *> itemsInScene = this->imageDisplay->scene()->items();
    foreach( QGraphicsItem *item, itemsInScene )
    {
      this->imageDisplay->scene()->removeItem(item);
    }

    QGraphicsPixmapItem *filePixmapItem = new QGraphicsPixmapItem(QPixmap(
        QString(filename.c_str())).scaled(this->imageDisplayWidth,
                                          this->imageDisplayHeight,
                                          Qt::KeepAspectRatio));
    this->imageDisplay->scene()->addItem(filePixmapItem);
  }
}

/////////////////////////////////////////////////
bool ImportImageDialog::eventFilter(QObject *obj, QEvent *_event)
{
  if (_event->type() == QEvent::Enter)
  {
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
    return true;
  }
  else if (_event->type() == QEvent::Leave)
  {
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    this->drawingLine = false;
    return true;
  }
  else if (_event->type() == QEvent::MouseButtonPress)
  {std::cout << "Mouse press" << std::endl;
    QMouseEvent *_mouseEvent = (QMouseEvent *)_event;
    this->measureLineStart = this->imageDisplay->mapToScene(_mouseEvent->pos());


    QGraphicsTextItem *Atext = new QGraphicsTextItem;
    Atext->setPlainText("AAA");
    Atext->setDefaultTextColor(QColor(255, 255, 255, 255));
    Atext->setPos(this->measureLineStart);
    this->imageDisplay->scene()->addItem(Atext);

    this->drawingLine = true;

    return true;
  }
  else if (_event->type() == QEvent::MouseMove)
  {
      std::cout << "Mouse move" << std::endl;


    if (this->drawingLine)
    {
        QMouseEvent *_mouseEvent = (QMouseEvent *)_event;

        QGraphicsLineItem *measureLine = new QGraphicsLineItem;
        measureLine->setPen(QPen(Qt::red,10));
        measureLine->setLine(this->measureLineStart.x(),
                             this->measureLineStart.y(),
                             this->imageDisplay->mapToScene(_mouseEvent->pos()).x(),
                             this->imageDisplay->mapToScene(_mouseEvent->pos()).y());
        this->imageDisplay->scene()->addItem(measureLine);
     }
     return true;
  }
  else if (_event->type() == QEvent::MouseButtonRelease)
  {std::cout << "Mouse released" << std::endl;

    this->drawingLine = false;

    return true;
  }
  else
  {
    // standard event processing
    return QObject::eventFilter(obj, _event);
  }
}
