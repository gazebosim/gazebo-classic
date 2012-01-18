/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <QtGui>

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/WorldPropertiesWidget.hh"

using namespace gazebo;
using namespace gui;

WorldPropertiesWidget::WorldPropertiesWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setWindowTitle("Gazebo: World");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QTabWidget *tabWidget = new QTabWidget;
  this->sceneWidget = new SceneWidget;
  this->physicsWidget = new PhysicsWidget;

  tabWidget->addTab(this->sceneWidget, tr("Scene"));
  tabWidget->addTab(this->physicsWidget, tr("Physics"));

  mainLayout->addWidget(tabWidget);

  this->setLayout(mainLayout);
  this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

WorldPropertiesWidget::~WorldPropertiesWidget()
{
}

void WorldPropertiesWidget::showEvent(QShowEvent * /*_event*/)
{
  this->sceneWidget->Init();
  this->physicsWidget->Init();
}

void WorldPropertiesWidget::closeEvent(QCloseEvent * /*_event*/)
{
  this->sceneWidget->initialized = false;
}

PhysicsWidget::PhysicsWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->initialized = false;

  this->setWindowTitle("Gazebo: Physics");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->setLayout(mainLayout);

  QHBoxLayout *gravityLayout = new QHBoxLayout;
  this->gravityXLineEdit = new QLineEdit;
  this->gravityXLineEdit->setValidator(
      new QDoubleValidator(this->gravityXLineEdit));
  this->gravityXLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->gravityXLineEdit->setFixedWidth(50);

  this->gravityYLineEdit = new QLineEdit;
  this->gravityYLineEdit->setValidator(
      new QDoubleValidator(this->gravityYLineEdit));
  this->gravityYLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->gravityYLineEdit->setFixedWidth(50);

  this->gravityZLineEdit = new QLineEdit;
  this->gravityZLineEdit->setValidator(
      new QDoubleValidator(this->gravityZLineEdit));
  this->gravityZLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->gravityZLineEdit->setFixedWidth(50);

  QLabel *gravityLabel = new QLabel(tr("Gravity:"));
  gravityLayout->addWidget(gravityLabel);
  gravityLayout->addWidget(this->gravityXLineEdit);
  gravityLayout->addWidget(this->gravityYLineEdit);
  gravityLayout->addWidget(this->gravityZLineEdit);


  QGroupBox *solverBox = new QGroupBox(tr("Solver"));
  solverBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QVBoxLayout *solverBoxLayout = new QVBoxLayout;
  solverBox->setLayout(solverBoxLayout);


  QHBoxLayout *solverTypeLayout = new QHBoxLayout;
  this->solverTypeBox = new QComboBox();
  this->solverTypeBox->addItem(tr("quick"));
  this->solverTypeBox->addItem(tr("world"));
  QLabel *solverTypeLabel = new QLabel(tr("Type:"));
  solverTypeLayout->addWidget(solverTypeLabel);
  solverTypeLayout->addWidget(this->solverTypeBox);
  solverTypeLayout->insertStretch(1, 0);

  QHBoxLayout *dtLayout = new QHBoxLayout;
  this->dtLineEdit = new QLineEdit;
  this->dtLineEdit->setValidator(new QDoubleValidator(this->dtLineEdit));
  this->dtLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->dtLineEdit->setFixedWidth(80);
  QLabel *dtLabel = new QLabel(tr("Step Duration:"));
  dtLayout->addWidget(dtLabel);
  dtLayout->addWidget(this->dtLineEdit);
  dtLayout->insertStretch(1, 0);

  QHBoxLayout *itersLayout = new QHBoxLayout;
  this->itersLineEdit = new QLineEdit;
  this->itersLineEdit->setValidator(new QIntValidator(this->itersLineEdit));
  this->itersLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->itersLineEdit->setFixedWidth(80);
  QLabel *itersLabel = new QLabel(tr("Iterations:"));
  itersLayout->addWidget(itersLabel);
  itersLayout->addWidget(this->itersLineEdit);
  itersLayout->insertStretch(1, 0);

  QHBoxLayout *sorLayout = new QHBoxLayout;
  this->sorLineEdit = new QLineEdit;
  this->sorLineEdit->setValidator(new QDoubleValidator(this->sorLineEdit));
  this->sorLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->sorLineEdit->setFixedWidth(80);
  QLabel *sorLabel = new QLabel(tr("SOR:"));
  sorLayout->addWidget(sorLabel);
  sorLayout->addWidget(this->sorLineEdit);
  sorLayout->insertStretch(1, 0);

  solverBoxLayout->addLayout(solverTypeLayout);
  solverBoxLayout->addLayout(itersLayout);
  solverBoxLayout->addLayout(sorLayout);
  solverBoxLayout->addLayout(dtLayout);

  QGroupBox *constraintsBox = new QGroupBox(tr("Constraints"));
  constraintsBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QVBoxLayout *constraintsBoxLayout = new QVBoxLayout;
  constraintsBox->setLayout(constraintsBoxLayout);

  QHBoxLayout *cfmLayout = new QHBoxLayout;
  this->cfmLineEdit = new QLineEdit;
  this->cfmLineEdit->setValidator(new QDoubleValidator(this->cfmLineEdit));
  this->cfmLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->cfmLineEdit->setFixedWidth(80);
  QLabel *cfmLabel = new QLabel(tr("CFM:"));
  cfmLayout->addWidget(cfmLabel);
  cfmLayout->addWidget(this->cfmLineEdit);
  cfmLayout->insertStretch(1, 0);

  QHBoxLayout *erpLayout = new QHBoxLayout;
  this->erpLineEdit = new QLineEdit;
  this->erpLineEdit->setValidator(new QDoubleValidator(this->erpLineEdit));
  this->erpLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->erpLineEdit->setFixedWidth(80);
  QLabel *erpLabel = new QLabel(tr("ERP:"));
  erpLayout->addWidget(erpLabel);
  erpLayout->addWidget(this->erpLineEdit);
  erpLayout->insertStretch(1, 0);

  QHBoxLayout *maxVelLayout = new QHBoxLayout;
  this->maxVelLineEdit = new QLineEdit;
  this->maxVelLineEdit->setValidator(
      new QDoubleValidator(this->maxVelLineEdit));
  this->maxVelLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->maxVelLineEdit->setFixedWidth(80);
  QLabel *maxVelLabel = new QLabel(tr("Max Velocity:"));
  maxVelLayout->addWidget(maxVelLabel);
  maxVelLayout->addWidget(this->maxVelLineEdit);
  maxVelLayout->insertStretch(1, 0);

  QHBoxLayout *surfaceLayerLayout = new QHBoxLayout;
  this->surfaceLayerLineEdit = new QLineEdit;
  this->surfaceLayerLineEdit->setValidator(
      new QDoubleValidator(this->surfaceLayerLineEdit));
  this->surfaceLayerLineEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  this->surfaceLayerLineEdit->setFixedWidth(80);
  QLabel *surfaceLayerLabel = new QLabel(tr("Surface Layer:"));
  surfaceLayerLayout->addWidget(surfaceLayerLabel);
  surfaceLayerLayout->addWidget(this->surfaceLayerLineEdit);
  surfaceLayerLayout->insertStretch(1, 0);

  constraintsBoxLayout->addLayout(cfmLayout);
  constraintsBoxLayout->addLayout(erpLayout);
  constraintsBoxLayout->addLayout(maxVelLayout);
  constraintsBoxLayout->addLayout(surfaceLayerLayout);

  mainLayout->addLayout(gravityLayout);
  mainLayout->addWidget(solverBox);
  mainLayout->addWidget(constraintsBox);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->physicsPub = this->node->Advertise<msgs::Physics>("~/physics");

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
      &PhysicsWidget::OnResponse, this);

  this->requestMsg = NULL;

  connect(this->gravityXLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnGravity()));
  connect(this->gravityYLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnGravity()));
  connect(this->gravityZLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnGravity()));

  connect(this->solverTypeBox, SIGNAL(currentIndexChanged(int)),
           this, SLOT(OnSolverType(int)));
  connect(this->dtLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnDt()));
  connect(this->sorLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnSOR()));
  connect(this->itersLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnIters()));
  connect(this->cfmLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnCFM()));
  connect(this->erpLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnERP()));
  connect(this->maxVelLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnMaxVel()));
  connect(this->surfaceLayerLineEdit, SIGNAL(editingFinished()),
           this, SLOT(OnSurfaceLayer()));
}

PhysicsWidget::~PhysicsWidget()
{
}

void PhysicsWidget::Init()
{
  this->initialized = false;
  this->requestMsg = msgs::CreateRequest("physics_info");
  this->requestPub->Publish(*this->requestMsg);
}

void PhysicsWidget::OnResponse(
    ConstResponsePtr &_msg)
{
  if (this->initialized || !this->requestMsg ||
      this->requestMsg->id() != _msg->id())
  {
    return;
  }

  msgs::Physics physicsMsg;
  if (_msg->has_type() && _msg->type() == physicsMsg.GetTypeName())
  {
    physicsMsg.ParseFromString(_msg->serialized_data());

    if (physicsMsg.has_gravity())
    {
      std::ostringstream x, y, z;
      double xDbl = physicsMsg.gravity().x();
      double yDbl = physicsMsg.gravity().y();
      double zDbl = physicsMsg.gravity().z();

      x << std::fixed << std::setprecision(3) << xDbl;
      y << std::fixed << std::setprecision(3) << yDbl;
      z << std::fixed << std::setprecision(3) << zDbl;

      this->gravityXLineEdit->setText(tr(x.str().c_str()));
      this->gravityYLineEdit->setText(tr(y.str().c_str()));
      this->gravityZLineEdit->setText(tr(z.str().c_str()));
    }

    if (physicsMsg.has_iters())
    {
      std::ostringstream iters;
      iters << std::fixed << std::setprecision(0) << physicsMsg.iters();
      this->itersLineEdit->setText(tr(iters.str().c_str()));
    }
    else
      this->itersLineEdit->setText(tr("0"));

    if (physicsMsg.has_solver_type())
    {
      int index = this->solverTypeBox->findText(
          physicsMsg.solver_type().c_str());

      if (index >= 0)
        this->solverTypeBox->setCurrentIndex(index);
      else
        gzerr << "Unknown physics solver type\n";
    }

    if (physicsMsg.has_dt())
    {
      std::ostringstream dt;
      dt << std::fixed << std::setprecision(5) << physicsMsg.dt();
      this->dtLineEdit->setText(tr(dt.str().c_str()));
    }
    else
      this->dtLineEdit->setText(tr("0"));


    if (physicsMsg.has_sor())
    {
      std::ostringstream sor;
      sor << std::fixed << std::setprecision(3) << physicsMsg.sor();
      this->sorLineEdit->setText(tr(sor.str().c_str()));
    }
    else
      this->sorLineEdit->setText(tr("0"));

    if (physicsMsg.has_cfm())
    {
      std::ostringstream cfm;
      cfm << std::fixed << std::setprecision(3) << physicsMsg.cfm();
      this->cfmLineEdit->setText(tr(cfm.str().c_str()));
    }
    else
      this->cfmLineEdit->setText(tr("0"));

    if (physicsMsg.has_erp())
    {
      std::ostringstream erp;
      erp << std::fixed << std::setprecision(3) << physicsMsg.erp();
      this->erpLineEdit->setText(tr(erp.str().c_str()));
    }
    else
      this->erpLineEdit->setText(tr("0"));

    if (physicsMsg.has_contact_max_correcting_vel())
    {
      std::ostringstream max;
      max << std::fixed << std::setprecision(3)
          << physicsMsg.contact_max_correcting_vel();
      this->maxVelLineEdit->setText(tr(max.str().c_str()));
    }
    else
      this->maxVelLineEdit->setText(tr("0"));

    if (physicsMsg.has_contact_surface_layer())
    {
      std::ostringstream sur;
      sur << std::fixed << std::setprecision(3)
          << physicsMsg.contact_surface_layer();
      this->surfaceLayerLineEdit->setText(tr(sur.str().c_str()));
    }
    else
      this->surfaceLayerLineEdit->setText(tr("0"));

    this->initialized = true;
  }

  delete this->requestMsg;
  this->requestMsg = NULL;
}

void PhysicsWidget::OnSolverType(int _index)
{
  if (!this->initialized)
    return;

  std::string value = this->solverTypeBox->itemText(_index).toStdString();

  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_solver_type(value);

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnGravity()
{
  if (!this->initialized)
    return;

  std::string valueX = this->gravityXLineEdit->text().toStdString();
  std::string valueY = this->gravityYLineEdit->text().toStdString();
  std::string valueZ = this->gravityZLineEdit->text().toStdString();

  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.mutable_gravity()->set_x(boost::lexical_cast<double>(valueX));
  msg.mutable_gravity()->set_y(boost::lexical_cast<double>(valueY));
  msg.mutable_gravity()->set_z(boost::lexical_cast<double>(valueZ));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnDt()
{
  if (!this->initialized)
    return;

  std::string value = this->dtLineEdit->text().toStdString();
  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_dt(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnSOR()
{
  if (!this->initialized)
    return;

  std::string value = this->sorLineEdit->text().toStdString();
  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_sor(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnIters()
{
  if (!this->initialized)
    return;

  std::string value = this->itersLineEdit->text().toStdString();

  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_iters(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnCFM()
{
  if (!this->initialized)
    return;

  std::string value = this->cfmLineEdit->text().toStdString();
  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_cfm(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnERP()
{
  if (!this->initialized)
    return;

  std::string value = this->erpLineEdit->text().toStdString();
  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_erp(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnMaxVel()
{
  if (!this->initialized)
    return;
  std::string value = this->maxVelLineEdit->text().toStdString();

  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_contact_max_correcting_vel(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}

void PhysicsWidget::OnSurfaceLayer()
{
  if (!this->initialized)
    return;

  std::string value = this->surfaceLayerLineEdit->text().toStdString();

  msgs::Physics msg;
  msg.set_type(msgs::Physics::ODE);
  msg.set_contact_surface_layer(boost::lexical_cast<double>(value));

  this->physicsPub->Publish(msg);
}




SceneWidget::SceneWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->initialized = false;

  this->setWindowTitle("Gazebo: Scene");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QGroupBox *ambientBox = new QGroupBox(tr("Ambient"));
  ambientBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QHBoxLayout *ambientLayout = new QHBoxLayout;
  this->ambientColorButton = new QPushButton;
  QLabel *ambientColorLabel = new QLabel(tr("Color:"));
  ambientLayout->addWidget(ambientColorLabel);
  ambientLayout->addWidget(this->ambientColorButton);
  ambientLayout->insertStretch(1, 0);
  ambientBox->setLayout(ambientLayout);


  QGroupBox *backgroundBox = new QGroupBox(tr("Background"));
  backgroundBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QHBoxLayout *backgroundLayout = new QHBoxLayout;
  this->backgroundColorButton = new QPushButton;
  QLabel *backgroundColorLabel = new QLabel(tr("Color:"));
  backgroundLayout->addWidget(backgroundColorLabel);
  backgroundLayout->addWidget(this->backgroundColorButton);
  backgroundLayout->insertStretch(1, 0);
  backgroundBox->setLayout(backgroundLayout);

  QGroupBox *shadowsBox = new QGroupBox(tr("Shadows"));
  shadowsBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QVBoxLayout *shadowsBoxLayout = new QVBoxLayout;
  this->shadowsButton = new QCheckBox(tr("Enabled"));
  this->shadowsButton->setCheckState(Qt::Checked);
  shadowsBoxLayout->addWidget(this->shadowsButton);
  shadowsBox->setLayout(shadowsBoxLayout);

  this->fogBox = new QGroupBox(tr("Fog"));
  this->fogBox->setCheckable(true);
  this->fogBox->setStyleSheet(
      tr("QGroupBox{border: 1px solid black; padding-top: 2ex;}"));
  QVBoxLayout *fogBoxLayout = new QVBoxLayout;

  QHBoxLayout *fogTypeLayout = new QHBoxLayout;
  this->fogTypeBox = new QComboBox();
  this->fogTypeBox->addItem(tr("Linear"));
  this->fogTypeBox->addItem(tr("Exponential"));
  this->fogTypeBox->addItem(tr("Squared"));
  QLabel *fogTypeLabel = new QLabel(tr("Type:"));
  fogTypeLayout->addWidget(fogTypeLabel);
  fogTypeLayout->insertStretch(1, 0);
  fogTypeLayout->addWidget(this->fogTypeBox);

  QHBoxLayout *fogColorLayout = new QHBoxLayout;
  this->fogColorButton = new QPushButton;
  QLabel *fogColorLabel = new QLabel(tr("Color:"));
  fogColorLayout->addWidget(fogColorLabel);
  fogColorLayout->addWidget(this->fogColorButton);
  fogColorLayout->insertStretch(1, 0);

  QHBoxLayout *fogStartLayout = new QHBoxLayout;
  this->fogStart = new QLineEdit;
  this->fogStart->setValidator(new QDoubleValidator(this->fogStart));
  this->fogStart->setInputMethodHints(Qt::ImhDigitsOnly);
  this->fogStart->setFixedWidth(80);
  QLabel *fogStartLabel = new QLabel(tr("Start:"));
  fogStartLayout->addWidget(fogStartLabel);
  fogStartLayout->addWidget(fogStart);
  fogStartLayout->insertStretch(1, 0);

  QHBoxLayout *fogEndLayout = new QHBoxLayout;
  this->fogEnd = new QLineEdit;
  this->fogEnd->setValidator(new QDoubleValidator(this->fogEnd));
  this->fogEnd->setInputMethodHints(Qt::ImhDigitsOnly);
  this->fogEnd->setFixedWidth(80);
  QLabel *fogEndLabel = new QLabel(tr("End:"));
  fogEndLayout->addWidget(fogEndLabel);
  fogEndLayout->addWidget(fogEnd);
  fogEndLayout->insertStretch(1, 0);


  QHBoxLayout *fogDensityLayout = new QHBoxLayout;
  this->fogDensitySpin = new QDoubleSpinBox;
  this->fogDensitySpin->setRange(0.0, 1.0);
  this->fogDensitySpin->setSingleStep(0.01);
  QLabel *fogDensityLabel = new QLabel(tr("Density:"));
  fogDensityLayout->addWidget(fogDensityLabel);
  fogDensityLayout->addItem(new QSpacerItem(40, 20,
        QSizePolicy::Expanding, QSizePolicy::Minimum));
  fogDensityLayout->addWidget(this->fogDensitySpin);
  fogDensityLayout->insertStretch(1, 0);


  fogBoxLayout->addLayout(fogColorLayout);
  fogBoxLayout->addLayout(fogTypeLayout);
  fogBoxLayout->addLayout(fogStartLayout);
  fogBoxLayout->addLayout(fogEndLayout);
  fogBoxLayout->addLayout(fogDensityLayout);

  this->fogBox->setLayout(fogBoxLayout);

  mainLayout->addWidget(shadowsBox);
  mainLayout->addWidget(ambientBox);
  mainLayout->addWidget(backgroundBox);
  mainLayout->addWidget(this->fogBox);
  mainLayout->insertStretch(4, 0);

  this->setLayout(mainLayout);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
      &SceneWidget::OnResponse, this);

  connect(this->ambientColorButton, SIGNAL(clicked()),
           this, SLOT(OnAmbientColor()));
  connect(this->backgroundColorButton, SIGNAL(clicked()),
           this, SLOT(OnBackgroundColor()));
  connect(this->fogColorButton, SIGNAL(clicked()),
           this, SLOT(OnFogColor()));

  connect(this->shadowsButton, SIGNAL(toggled(bool)),
           this, SLOT(OnShadows(bool)));

  connect(this->fogStart, SIGNAL(editingFinished()),
           this, SLOT(OnFogStart()));
  connect(this->fogEnd, SIGNAL(editingFinished()),
           this, SLOT(OnFogEnd()));
  connect(this->fogDensitySpin, SIGNAL(valueChanged(double)),
           this, SLOT(OnFogDensity(double)));
  connect(this->fogTypeBox, SIGNAL(currentIndexChanged(int)),
           this, SLOT(OnFogType(int)));


  connect(this->fogBox, SIGNAL(toggled(bool)),
           this, SLOT(OnFogToggle(bool)));

  this->requestMsg = NULL;
}

SceneWidget::~SceneWidget()
{
  delete this->requestMsg;
  this->requestMsg = NULL;
}

void SceneWidget::Init()
{
  this->requestMsg = msgs::CreateRequest("scene_info");
  this->requestPub->Publish(*this->requestMsg);
  this->initialized = false;
}

void SceneWidget::OnFogToggle(bool _value)
{
  if (!this->initialized)
    return;

  msgs::Scene msg;
  if (_value)
  {
    int index = this->fogTypeBox->currentIndex();
    this->OnFogType(index);
  }
  else
    msg.mutable_fog()->set_type(msgs::Fog::NONE);

  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogType(int _index)
{
  if (!this->initialized)
    return;
  msgs::Scene msg;

  if (_index == 0)
    msg.mutable_fog()->set_type(msgs::Fog::LINEAR);
  else if (_index == 1)
    msg.mutable_fog()->set_type(msgs::Fog::EXPONENTIAL);
  else
    msg.mutable_fog()->set_type(msgs::Fog::EXPONENTIAL2);

  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogDensity(double _density)
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  msg.mutable_fog()->set_density(_density);

  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogStart()
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  std::string value = this->fogStart->text().toStdString();
  if (!value.empty())
  {
    msg.mutable_fog()->set_start(boost::lexical_cast<double>(value));

    msg.set_name(this->sceneName);
    this->scenePub->Publish(msg);
  }
}

void SceneWidget::OnFogEnd()
{
  if (!this->initialized)
    return;
  msgs::Scene msg;
  std::string value = this->fogEnd->text().toStdString();
  if (!value.empty())
  {
    msg.mutable_fog()->set_end(boost::lexical_cast<double>(value));

    msg.set_name(this->sceneName);
    this->scenePub->Publish(msg);
  }
}

void SceneWidget::OnAmbientColor()
{
  if (!this->initialized)
    return;

  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  std::ostringstream style;
  style << "background-color:rgb(" << color.red()
        << ", " << color.green() << ", " << color.blue() << ");";
  this->ambientColorButton->setAutoFillBackground(true);
  this->ambientColorButton->setStyleSheet(style.str().c_str());

  msgs::Scene msg;
  msg.mutable_ambient()->set_r(color.red() /255.0);
  msg.mutable_ambient()->set_g(color.green() /255.0);
  msg.mutable_ambient()->set_b(color.blue() /255.0);
  msg.mutable_ambient()->set_a(1.0);

  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnBackgroundColor()
{
  if (!this->initialized)
    return;

  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  std::ostringstream style;
  style << "background-color:rgb(" << color.red()
        << ", " << color.green() << ", " << color.blue() << ");";
  this->backgroundColorButton->setAutoFillBackground(true);
  this->backgroundColorButton->setStyleSheet(style.str().c_str());


  msgs::Scene msg;
  msg.mutable_background()->set_r(color.red() / 255.0);
  msg.mutable_background()->set_g(color.green() / 255.0);
  msg.mutable_background()->set_b(color.blue() / 255.0);
  msg.mutable_background()->set_a(1.0);
  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnFogColor()
{
  if (!this->initialized)
    return;

  QColor color;
  color = QColorDialog::getColor(Qt::yellow, this);

  std::ostringstream style;
  style << "background-color:rgb(" << color.red() << ", "
        << color.green() << ", " << color.blue() << ");";
  this->fogColorButton->setAutoFillBackground(true);
  this->fogColorButton->setStyleSheet(style.str().c_str());

  msgs::Scene msg;
  msg.mutable_fog()->mutable_color()->set_r(color.red() / 255.0);
  msg.mutable_fog()->mutable_color()->set_g(color.green() / 255.0);
  msg.mutable_fog()->mutable_color()->set_b(color.blue() / 255.0);
  msg.mutable_fog()->mutable_color()->set_a(1.0);
  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnShadows(bool _state)
{
  if (!this->initialized)
    return;

  msgs::Scene msg;
  msg.set_shadows(_state);
  msg.set_name(this->sceneName);
  this->scenePub->Publish(msg);
}

void SceneWidget::OnResponse(ConstResponsePtr &_msg)
{
  if (this->initialized || !this->requestMsg ||
      this->requestMsg->id() != _msg->id())
  {
    return;
  }

  msgs::Scene sceneMsg;
  if (_msg->has_type() && _msg->type() == sceneMsg.GetTypeName())
  {
    sceneMsg.ParseFromString(_msg->serialized_data());
    this->sceneName = sceneMsg.name();

    if (sceneMsg.has_ambient())
    {
      std::ostringstream style;
      style << "background-color:rgb(" << sceneMsg.ambient().r()*255
        << ", " << sceneMsg.ambient().g() * 255 << ", "
        << sceneMsg.ambient().b()*255 << ");";
      this->ambientColorButton->setAutoFillBackground(true);
      this->ambientColorButton->setStyleSheet(style.str().c_str());
    }

    if (sceneMsg.has_background())
    {
      std::ostringstream style;
      style << "background-color:rgb(" << sceneMsg.background().r()*255
        << ", " << sceneMsg.background().g() * 255 << ", "
        << sceneMsg.background().b()*255 << ");";
      this->backgroundColorButton->setAutoFillBackground(true);
      this->backgroundColorButton->setStyleSheet(style.str().c_str());
    }

    if (sceneMsg.has_shadows() && sceneMsg.shadows())
      this->shadowsButton->setCheckState(Qt::Checked);
    else
      this->shadowsButton->setCheckState(Qt::Unchecked);

    if (sceneMsg.has_fog() && sceneMsg.fog().type() != msgs::Fog::NONE)
    {
      this->fogBox->setChecked(true);

      if (sceneMsg.fog().has_color())
      {
        std::ostringstream style;
        style << "background-color:rgb("
          << sceneMsg.fog().color().r()*255
          << ", " << sceneMsg.fog().color().g() * 255 << ", "
          << sceneMsg.fog().color().b()*255 << ");";
        this->fogColorButton->setAutoFillBackground(true);
        this->fogColorButton->setStyleSheet(style.str().c_str());
      }

      if (sceneMsg.fog().has_density())
        this->fogDensitySpin->setValue(sceneMsg.fog().density());

      if (sceneMsg.fog().has_start())
      {
        this->fogStart->setText(
            tr(boost::lexical_cast<std::string>(
                sceneMsg.fog().start()).c_str()));
      }

      if (sceneMsg.fog().has_end())
      {
        this->fogEnd->setText(
            tr(boost::lexical_cast<std::string>(
                sceneMsg.fog().end()).c_str()));
      }
    }
    else
    {
      this->fogBox->setChecked(false);
    }
  }
  this->initialized = true;
}

