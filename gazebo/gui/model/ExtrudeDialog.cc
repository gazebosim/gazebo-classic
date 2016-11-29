/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <ignition/math/Vector2.hh>

#include "gazebo/common/SVGLoader.hh"

#include "gazebo/gui/model/ExtrudeDialogPrivate.hh"
#include "gazebo/gui/model/ExtrudeDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ExtrudeDialog::ExtrudeDialog(const std::string &_filename, QWidget *_parent)
  : QDialog(_parent), dataPtr(new ExtrudeDialogPrivate)
{
  this->setObjectName("ExtrudeDialog");
  this->setWindowTitle(tr("Extrude Link"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Title
  QLabel *titleLabel = new QLabel(tr(
      "Extrude a 2D polyline into a 3D mesh.<br>"));

  // Thickness
  this->dataPtr->thicknessSpin = new QDoubleSpinBox();
  this->dataPtr->thicknessSpin->setObjectName("thicknessSpin");
  this->dataPtr->thicknessSpin->setRange(0.001, 1000);
  this->dataPtr->thicknessSpin->setSingleStep(0.1);
  this->dataPtr->thicknessSpin->setDecimals(3);
  this->dataPtr->thicknessSpin->setValue(0.1);

  // Resolution
  this->dataPtr->resolutionSpin = new QDoubleSpinBox();
  this->dataPtr->resolutionSpin->setObjectName("resolutionSpin");
  this->dataPtr->resolutionSpin->setRange(1, 100000);
  this->dataPtr->resolutionSpin->setSingleStep(100);
  this->dataPtr->resolutionSpin->setDecimals(3);
  // 3543.3 px/m == 90 dpi
  this->dataPtr->resolutionSpin->setValue(3543.3);
  connect(this->dataPtr->resolutionSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnUpdateView(double)));

  // Samples
  this->dataPtr->samplesSpin = new QSpinBox();
  this->dataPtr->samplesSpin->setObjectName("samplesSpin");
  this->dataPtr->samplesSpin->setRange(2, 100);
  this->dataPtr->samplesSpin->setSingleStep(1);
  this->dataPtr->samplesSpin->setValue(5);
  QLabel *samplesTips = new QLabel(tr("<b><font size=4>?</font></b>"));
  samplesTips->setToolTip(
      "Number of points to divide each curve segment into.");
  connect(this->dataPtr->samplesSpin, SIGNAL(valueChanged(int)),
      this, SLOT(OnUpdateView(int)));

  QGridLayout *inputsLayout = new QGridLayout();
  inputsLayout->addWidget(new QLabel("Thickness:"), 0, 0);
  inputsLayout->addWidget(this->dataPtr->thicknessSpin, 0, 1);
  inputsLayout->addWidget(new QLabel("m"), 0, 2);
  inputsLayout->addWidget(new QLabel("Resolution:"), 1, 0);
  inputsLayout->addWidget(this->dataPtr->resolutionSpin, 1, 1);
  inputsLayout->addWidget(new QLabel("px/m"), 1, 2);
  inputsLayout->addWidget(new QLabel("Samples per segment:"), 2, 0);
  inputsLayout->addWidget(this->dataPtr->samplesSpin, 2, 1);
  inputsLayout->addWidget(samplesTips, 2, 2);

  // Buttons
  QPushButton *backButton = new QPushButton(tr("Back"));
  connect(backButton, SIGNAL(clicked()), this, SLOT(OnReject()));

  QPushButton *okButton = new QPushButton("Ok");
  okButton->setDefault(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(OnAccept()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  buttonsLayout->addWidget(backButton);
  buttonsLayout->addWidget(okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Left column
  QWidget *leftColumn = new QWidget();
  leftColumn->setSizePolicy(QSizePolicy::Fixed,
                            QSizePolicy::Fixed);
  QVBoxLayout *leftColumnLayout = new QVBoxLayout();
  leftColumn->setLayout(leftColumnLayout);
  leftColumnLayout->addWidget(titleLabel);
  leftColumnLayout->addLayout(inputsLayout);
  leftColumnLayout->addSpacing(30);
  leftColumnLayout->addLayout(buttonsLayout);

  // Image view
  this->dataPtr->filename = _filename;

  this->dataPtr->view = new QGraphicsView(this);
  QGraphicsScene *scene = new QGraphicsScene();
  scene->setBackgroundBrush(Qt::white);
  this->dataPtr->view->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->dataPtr->view->setScene(scene);
  this->dataPtr->view->setViewportUpdateMode(
      QGraphicsView::FullViewportUpdate);
  this->dataPtr->view->setDragMode(QGraphicsView::ScrollHandDrag);
  this->dataPtr->view->setMinimumWidth(200);
  this->dataPtr->viewWidth = 500;
  this->dataPtr->view->installEventFilter(this);
  this->UpdateView();

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftColumn, 0, Qt::AlignTop);
  mainLayout->addWidget(this->dataPtr->view);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ExtrudeDialog::~ExtrudeDialog()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ExtrudeDialog::OnAccept()
{
  this->accept();
}

/////////////////////////////////////////////////
void ExtrudeDialog::OnReject()
{
  // back to import
  this->reject();
}

/////////////////////////////////////////////////
double ExtrudeDialog::GetThickness() const
{
  return this->dataPtr->thicknessSpin->value();
}

/////////////////////////////////////////////////
unsigned int ExtrudeDialog::GetSamples() const
{
  return this->dataPtr->samplesSpin->value();
}

/////////////////////////////////////////////////
double ExtrudeDialog::GetResolution() const
{
  return this->dataPtr->resolutionSpin->value();
}

/////////////////////////////////////////////////
void ExtrudeDialog::OnUpdateView(int /*_value*/)
{
  this->UpdateView();
}

/////////////////////////////////////////////////
void ExtrudeDialog::OnUpdateView(double /*_value*/)
{
  this->UpdateView();
}

/////////////////////////////////////////////////
void ExtrudeDialog::UpdateView()
{
  QGraphicsScene *scene = this->dataPtr->view->scene();
  scene->clear();

  common::SVGLoader svgLoader(this->GetSamples());
  std::vector<common::SVGPath> paths;
  svgLoader.Parse(this->dataPtr->filename, paths);

  if (paths.empty())
  {
    gzerr << "An empty path should never get here." << std::endl;
    return;
  }

  // Find extreme values to center and scale
  ignition::math::Vector2d min(paths[0].polylines[0][0]);
  ignition::math::Vector2d max(min);
  for (const auto &p : paths)
  {
    for (const auto &poly : p.polylines)
    {
      for (const auto &pt : poly)
      {
        if (pt.X() < min.X())
          min.X(pt.X());
        if (pt.Y() < min.Y())
          min.Y(pt.Y());
        if (pt.X() > max.X())
          max.X(pt.X());
        if (pt.Y() > max.Y())
          max.Y(pt.Y());
      }
    }
  }

  int margin = 50;
  double svgWidth = this->dataPtr->viewWidth - margin * 2;
  double resolutionView = svgWidth / (max.X()-min.X());
  double svgHeight = (max.Y() - min.Y()) * resolutionView;
  double viewHeight = svgHeight + 2 * margin;
  scene->setSceneRect(0, 0, this->dataPtr->viewWidth, viewHeight);

  // Draw grid lines
  double sceneMeter = 1 * this->GetResolution() * resolutionView;
  for (double r = 0; r <= viewHeight; r += sceneMeter/10.0)
  {
    scene->addLine(- margin, r, this->dataPtr->viewWidth + margin, r,
        QPen(QColor(230, 230, 255)));
  }
  for (double c = 0; c <= this->dataPtr->viewWidth; c += sceneMeter/10.0)
  {
    scene->addLine(c, - margin, c, viewHeight + margin,
        QPen(QColor(230, 230, 255)));
  }
  for (double r = 0; r <= viewHeight; r += sceneMeter)
  {
    scene->addLine(- margin, r, this->dataPtr->viewWidth + margin, r,
        QPen(QColor(108, 108, 255)));
  }
  for (double c = 0; c <= this->dataPtr->viewWidth; c += sceneMeter)
  {
    scene->addLine(c, - margin, c, viewHeight + margin,
        QPen(QColor(108, 108, 255)));
  }

  // Draw origin
  scene->addLine(this->dataPtr->viewWidth/2.0 - this->dataPtr->viewWidth/30.0,
                 viewHeight/2.0,
                 this->dataPtr->viewWidth/2.0 + this->dataPtr->viewWidth/30.0,
                 viewHeight/2.0,
                 QPen(QColor(50, 50, 255), 2));
  scene->addLine(this->dataPtr->viewWidth/2.0,
                 viewHeight/2.0 - this->dataPtr->viewWidth/30.0,
                 this->dataPtr->viewWidth/2.0,
                 viewHeight/2.0 + this->dataPtr->viewWidth/30.0,
                 QPen(QColor(50, 50, 255), 2));

  // Draw polygons
  for (common::SVGPath path : paths)
  {
    for (std::vector<ignition::math::Vector2d> poly : path.polylines)
    {
      QPainterPath painterPath;
      bool firstPoint = true;
      for (ignition::math::Vector2d pt : poly)
      {
        // Centroid at SVG 0,0
        pt = pt - min - (max-min)*0.5;
        // Scale to view while keeping aspect ratio
        pt = ignition::math::Vector2d(
            pt.X() * resolutionView, pt.Y() * resolutionView);
        // Translate to view center
        pt.X() += this->dataPtr->viewWidth/2.0;
        pt.Y() += viewHeight/2.0;

        // Draw point
        double pointSize = 5;
        QGraphicsEllipseItem *ptItem = new QGraphicsEllipseItem(
             pt.X() - pointSize/2.0, pt.Y() - pointSize/2.0,
             pointSize, pointSize);
        ptItem->setBrush(Qt::red);
        ptItem->setZValue(5);
        scene->addItem(ptItem);
        if (firstPoint)
        {
          firstPoint = false;
          painterPath.moveTo(pt.X(), pt.Y());
        }
        else
        painterPath.lineTo(pt.X(), pt.Y());
      }

      QGraphicsPathItem *pathItem = new QGraphicsPathItem(painterPath);
      // Draw polygon
      pathItem->setPen(QPen(Qt::black, 3, Qt::SolidLine));
      scene->addItem(pathItem);
    }
  }
}

/////////////////////////////////////////////////
bool ExtrudeDialog::eventFilter(QObject *_obj, QEvent *_event)
{
  QGraphicsView *graphicsView = qobject_cast<QGraphicsView *>(_obj);
  if (graphicsView && _event->type() == QEvent::Resize)
  {
    QResizeEvent *resizeEv = dynamic_cast<QResizeEvent *>(_event);
    this->dataPtr->viewWidth = resizeEv->size().width() - 20;
    this->UpdateView();
  }
  return QObject::eventFilter(_obj, _event);
}
