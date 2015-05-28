/*
 * Copyright 2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"

#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ApplyWrenchDialogPrivate.hh"
#include "gazebo/gui/ApplyWrenchDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ApplyWrenchDialog::ApplyWrenchDialog(QWidget *_parent)
  : QDialog(_parent), dataPtr(new ApplyWrenchDialogPrivate)
{
  this->setObjectName("ApplyWrenchDialog");

  this->setWindowTitle(tr("Apply Force and Torque"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);
  this->setWindowModality(Qt::NonModal);
  this->setStyleSheet(
      "QPushButton {\
          border-radius: 5px;\
          border-radius: 5px;\
      }");

  this->dataPtr->modelLabel = new QLabel();

  // Links list
  QHBoxLayout *linkLayout = new QHBoxLayout();
  QLabel *linkLabel = new QLabel(tr("<b>Apply to link:<b> "));
  this->dataPtr->linksComboBox = new QComboBox();
  this->dataPtr->linksComboBox->installEventFilter(this);
  this->dataPtr->linksComboBox->setMinimumWidth(200);
  connect(this->dataPtr->linksComboBox, SIGNAL(currentIndexChanged(QString)),
      this, SLOT(SetLink(QString)));

  linkLayout->addWidget(linkLabel);
  linkLayout->addWidget(this->dataPtr->linksComboBox);

  // Force
  QLabel *forceLabel = new QLabel(tr(
       "<font size=4>Force</font>"));
  forceLabel->setObjectName("forceLabel");
  forceLabel->setStyleSheet(
      "QLabel#forceLabel {\
          background-color: #444;\
          border-radius: 5px;\
          padding-left: 10px;\
          min-height: 40px;\
      }");

  // Force vector layout
  QGridLayout *forceVectorLayout = new QGridLayout();

  // Force Vector
  this->dataPtr->forceXSpin = new QDoubleSpinBox();
  this->dataPtr->forceYSpin = new QDoubleSpinBox();
  this->dataPtr->forceZSpin = new QDoubleSpinBox();

  std::vector<QDoubleSpinBox *> forceSpins;
  forceSpins.push_back(this->dataPtr->forceXSpin);
  forceSpins.push_back(this->dataPtr->forceYSpin);
  forceSpins.push_back(this->dataPtr->forceZSpin);

  for (unsigned int i = 0; i < forceSpins.size(); ++i)
  {
    QLabel *forceElementLabel = new QLabel();
    if (i == 0)
      forceElementLabel->setText(tr("X:"));
    else if (i == 1)
      forceElementLabel->setText(tr("Y:"));
    else if (i == 2)
      forceElementLabel->setText(tr("Z:"));
    QLabel *forceUnitLabel = new QLabel(tr("N"));

    forceSpins[i]->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
    forceSpins[i]->setSingleStep(100);
    forceSpins[i]->setDecimals(3);
    forceSpins[i]->setValue(0);
    forceSpins[i]->setMaximumWidth(100);
    forceSpins[i]->installEventFilter(this);
    connect(forceSpins[i], SIGNAL(valueChanged(double)), this,
        SLOT(OnForceChanged(double)));

    forceVectorLayout->addWidget(forceElementLabel, i, 0, Qt::AlignRight);
    forceVectorLayout->addWidget(forceSpins[i], i, 1);
    forceVectorLayout->addWidget(forceUnitLabel, i, 2);
  }

  // Force total
  QLabel *forceMagLabel = new QLabel(tr("Mag:"));
  QLabel *forceMagUnitLabel = new QLabel(tr("N"));

  this->dataPtr->forceMagSpin = new QDoubleSpinBox();
  this->dataPtr->forceMagSpin->setRange(0, GZ_DBL_MAX);
  this->dataPtr->forceMagSpin->setSingleStep(100);
  this->dataPtr->forceMagSpin->setDecimals(3);
  this->dataPtr->forceMagSpin->setValue(0);
  this->dataPtr->forceMagSpin->setMaximumWidth(100);
  this->dataPtr->forceMagSpin->installEventFilter(this);
  connect(this->dataPtr->forceMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceMagChanged(double)));

  forceVectorLayout->addWidget(forceMagLabel, 3, 0, Qt::AlignRight);
  forceVectorLayout->addWidget(this->dataPtr->forceMagSpin, 3, 1);
  forceVectorLayout->addWidget(forceMagUnitLabel, 3, 2);

  // Clear force
  QPushButton *forceClearButton = new QPushButton(tr("Clear"));
  connect(forceClearButton, SIGNAL(clicked()), this, SLOT(OnForceClear()));
  forceVectorLayout->addWidget(forceClearButton, 4, 0, 1, 3, Qt::AlignLeft);

  // Vertical separator
  QFrame *separator = new QFrame();
  separator->setFrameShape(QFrame::VLine);
  separator->setLineWidth(10);

  // Force Position
  QLabel *forcePosLabel = new QLabel(tr("Application Point:"));
  forcePosLabel->setObjectName("forcePosLabel");
  forcePosLabel->setStyleSheet(
      "QLabel#forcePosLabel {\
          max-height: 15px;\
      }");

  // CoM
  QLabel *comLabel = new QLabel(tr("Center of mass"));

  QLabel *comPixLabel = new QLabel();
  QPixmap comPixmap(":images/com.png");
  comPixmap = comPixmap.scaled(QSize(20, 20));
  comPixLabel->setPixmap(comPixmap);
  comPixLabel->setMask(comPixmap.mask());

  QHBoxLayout *comLabelLayout = new QHBoxLayout();
  comLabelLayout->addWidget(comLabel);
  comLabelLayout->addWidget(comPixLabel);

  this->dataPtr->comRadio = new QRadioButton();
  this->dataPtr->forcePosRadio = new QRadioButton();
  this->dataPtr->comRadio->setChecked(true);
  connect(this->dataPtr->comRadio, SIGNAL(toggled(bool)), this,
      SLOT(ToggleComRadio(bool)));

  // Force Position layout
  QGridLayout *forcePosLayout = new QGridLayout();
  forcePosLayout->setContentsMargins(0, 0, 0, 0);
  forcePosLayout->addWidget(forcePosLabel, 0, 0, 1, 4, Qt::AlignLeft);
  forcePosLayout->addWidget(this->dataPtr->comRadio, 1, 0);
  forcePosLayout->addLayout(comLabelLayout, 1, 1, 1, 3, Qt::AlignLeft);
  forcePosLayout->addWidget(this->dataPtr->forcePosRadio, 2, 0);

  // Force Position Vector
  this->dataPtr->forcePosXSpin = new QDoubleSpinBox();
  this->dataPtr->forcePosYSpin = new QDoubleSpinBox();
  this->dataPtr->forcePosZSpin = new QDoubleSpinBox();

  std::vector<QDoubleSpinBox *> forcePosSpins;
  forcePosSpins.push_back(this->dataPtr->forcePosXSpin);
  forcePosSpins.push_back(this->dataPtr->forcePosYSpin);
  forcePosSpins.push_back(this->dataPtr->forcePosZSpin);

  for (unsigned int i = 0; i < forcePosSpins.size(); ++i)
  {
    QLabel *forcePosElementLabel = new QLabel();
    if (i == 0)
      forcePosElementLabel->setText(tr("X:"));
    else if (i == 1)
      forcePosElementLabel->setText(tr("Y:"));
    else if (i == 2)
      forcePosElementLabel->setText(tr("Z:"));
    QLabel *forcePosUnitLabel = new QLabel(tr("m"));

    forcePosSpins[i]->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
    forcePosSpins[i]->setSingleStep(0.1);
    forcePosSpins[i]->setDecimals(3);
    forcePosSpins[i]->setValue(0);
    forcePosSpins[i]->setMaximumWidth(100);
    forcePosSpins[i]->installEventFilter(this);
    connect(forcePosSpins[i], SIGNAL(valueChanged(double)), this,
        SLOT(OnForcePosChanged(double)));

    forcePosLayout->addWidget(forcePosElementLabel, i+2, 1, Qt::AlignRight);
    forcePosLayout->addWidget(forcePosSpins[i], i+2, 2);
    forcePosLayout->addWidget(forcePosUnitLabel, i+2, 3);
  }

  // Apply force
  QPushButton *applyForceButton = new QPushButton("Apply Force");
  connect(applyForceButton, SIGNAL(clicked()), this, SLOT(OnApplyForce()));

  // Force layout
  QGridLayout *forceLayout = new QGridLayout();
  forceLayout->setContentsMargins(0, 0, 0, 0);
  forceLayout->addWidget(forceLabel, 0, 0, 1, 5);
  forceLayout->addItem(new QSpacerItem(10, 10), 1, 0, 1, 5);
  forceLayout->addLayout(forceVectorLayout, 2, 1);
  forceLayout->addWidget(separator, 2, 2);
  forceLayout->addLayout(forcePosLayout, 2, 3);
  forceLayout->addItem(new QSpacerItem(10, 10), 3, 0, 1, 5);
  forceLayout->addWidget(applyForceButton, 4, 1, 1, 3, Qt::AlignRight);
  forceLayout->addItem(new QSpacerItem(5, 10), 5, 0);
  forceLayout->addItem(new QSpacerItem(7, 10), 5, 4);

  QFrame *forceFrame = new QFrame();
  forceFrame->setLayout(forceLayout);
  forceFrame->setObjectName("forceLayout");
  forceFrame->setFrameShape(QFrame::StyledPanel);

  forceFrame->setStyleSheet(
      "QFrame#forceLayout {\
          background-color: #666;\
          border-radius: 10px;\
      }");

  QGraphicsDropShadowEffect *forceEffect = new QGraphicsDropShadowEffect;
  forceEffect->setBlurRadius(5);
  forceEffect->setXOffset(5);
  forceEffect->setYOffset(5);
  forceEffect->setColor(Qt::black);
  forceFrame->setGraphicsEffect(forceEffect);

  // Torque
  QLabel *torqueLabel = new QLabel(tr("<font size=4>Torque</font>"));
  torqueLabel->setObjectName("torqueLabel");
  torqueLabel->setStyleSheet(
      "QLabel#torqueLabel {\
          background-color: #444;\
          border-radius: 5px;\
          padding-left: 10px;\
          min-height: 40px;\
      }");

  // Torque vector layout
  QGridLayout *torqueVectorLayout = new QGridLayout();

  // Torque Vector
  this->dataPtr->torqueXSpin = new QDoubleSpinBox();
  this->dataPtr->torqueYSpin = new QDoubleSpinBox();
  this->dataPtr->torqueZSpin = new QDoubleSpinBox();

  std::vector<QDoubleSpinBox *> torqueSpins;
  torqueSpins.push_back(this->dataPtr->torqueXSpin);
  torqueSpins.push_back(this->dataPtr->torqueYSpin);
  torqueSpins.push_back(this->dataPtr->torqueZSpin);

  for (unsigned int i = 0; i < torqueSpins.size(); ++i)
  {
    QLabel *torqueElementLabel = new QLabel();
    if (i == 0)
      torqueElementLabel->setText(tr("X:"));
    else if (i == 1)
      torqueElementLabel->setText(tr("Y:"));
    else if (i == 2)
      torqueElementLabel->setText(tr("Z:"));
    QLabel *torqueUnitLabel = new QLabel(tr("Nm"));

    torqueSpins[i]->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
    torqueSpins[i]->setSingleStep(100);
    torqueSpins[i]->setDecimals(3);
    torqueSpins[i]->setValue(0);
    torqueSpins[i]->setMaximumWidth(100);
    torqueSpins[i]->installEventFilter(this);
    connect(torqueSpins[i], SIGNAL(valueChanged(double)), this,
        SLOT(OnTorqueChanged(double)));

    torqueVectorLayout->addWidget(torqueElementLabel, i, 0, Qt::AlignRight);
    torqueVectorLayout->addWidget(torqueSpins[i], i, 1);
    torqueVectorLayout->addWidget(torqueUnitLabel, i, 2);
  }

  // Torque magnitude
  QLabel *torqueMagLabel = new QLabel(tr("Mag:"));
  QLabel *torqueMagUnitLabel = new QLabel(tr("Nm"));

  this->dataPtr->torqueMagSpin = new QDoubleSpinBox();
  this->dataPtr->torqueMagSpin->setRange(0, GZ_DBL_MAX);
  this->dataPtr->torqueMagSpin->setSingleStep(100);
  this->dataPtr->torqueMagSpin->setDecimals(3);
  this->dataPtr->torqueMagSpin->setValue(0);
  this->dataPtr->torqueMagSpin->setMaximumWidth(100);
  this->dataPtr->torqueMagSpin->installEventFilter(this);
  connect(this->dataPtr->torqueMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueMagChanged(double)));

  torqueVectorLayout->addWidget(torqueMagLabel, 3, 0, Qt::AlignRight);
  torqueVectorLayout->addWidget(this->dataPtr->torqueMagSpin, 3, 1);
  torqueVectorLayout->addWidget(torqueMagUnitLabel, 3, 2);

  // Clear torque
  QPushButton *torqueClearButton = new QPushButton(tr("Clear"));
  connect(torqueClearButton, SIGNAL(clicked()), this, SLOT(OnTorqueClear()));
  torqueVectorLayout->addWidget(torqueClearButton, 4, 0, 1, 3, Qt::AlignLeft);

  // Apply torque
  QPushButton *applyTorqueButton = new QPushButton("Apply Torque");
  connect(applyTorqueButton, SIGNAL(clicked()), this, SLOT(OnApplyTorque()));

  // Torque layout
  QGridLayout *torqueLayout = new QGridLayout();
  torqueLayout->setContentsMargins(0, 0, 0, 0);
  torqueLayout->addWidget(torqueLabel, 0, 0, 1, 3);
  torqueLayout->addItem(new QSpacerItem(10, 10), 1, 0, 1, 3);
  torqueLayout->addLayout(torqueVectorLayout, 2, 1);
  torqueLayout->addItem(new QSpacerItem(10, 10), 3, 0, 1, 3);
  torqueLayout->addWidget(applyTorqueButton, 4, 1, 1, 1, Qt::AlignRight);
  torqueLayout->addItem(new QSpacerItem(5, 10), 5, 0);
  torqueLayout->addItem(new QSpacerItem(5, 10), 5, 2);

  QFrame *torqueFrame = new QFrame();
  torqueFrame->setLayout(torqueLayout);
  torqueFrame->setObjectName("torqueLayout");
  torqueFrame->setFrameShape(QFrame::StyledPanel);

  torqueFrame->setStyleSheet(
      "QFrame#torqueLayout {\
          background-color: #666;\
          border-radius: 10px;\
      }");

  QGraphicsDropShadowEffect *torqueEffect = new QGraphicsDropShadowEffect;
  torqueEffect->setBlurRadius(5);
  torqueEffect->setXOffset(5);
  torqueEffect->setYOffset(5);
  torqueEffect->setColor(Qt::black);
  torqueFrame->setGraphicsEffect(torqueEffect);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *applyAllButton = new QPushButton("Apply All");
  applyAllButton->setDefault(true);
  connect(applyAllButton, SIGNAL(clicked()), this, SLOT(OnApplyAll()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyAllButton);

  // Main layout
  QGridLayout *mainLayout = new QGridLayout();
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  mainLayout->addWidget(this->dataPtr->modelLabel, 0, 0, 1, 2, Qt::AlignLeft);
  mainLayout->addLayout(linkLayout, 1, 0, 1, 2, Qt::AlignLeft);
  mainLayout->addWidget(forceFrame, 2, 0);
  mainLayout->addWidget(torqueFrame, 2, 1);
  mainLayout->addLayout(buttonsLayout, 3, 0, 1, 2, Qt::AlignRight);

  this->setLayout(mainLayout);

  // Transport
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->comVector = math::Vector3::Zero;
  this->dataPtr->forceVector = math::Vector3::Zero;
  this->dataPtr->torqueVector = math::Vector3::Zero;
}

/////////////////////////////////////////////////
ApplyWrenchDialog::~ApplyWrenchDialog()
{
  this->Fini();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::Init(const std::string &_modelName,
    const std::string &_linkName)
{
  if (!this->SetModel(_modelName))
  {
    this->Fini();
    return;
  }

  if (!this->SetLink(_linkName))
  {
    this->Fini();
    return;
  }

  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

  this->move(QCursor::pos());
  this->show();
  this->ActivateWindow();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::Fini()
{
  this->dataPtr->wrenchPub.reset();
  this->dataPtr->node->Fini();
  this->dataPtr->connections.clear();

  if (this->dataPtr->applyWrenchVisual)
  {
    MouseEventHandler::Instance()->RemoveReleaseFilter(
        "dialog_"+this->dataPtr->applyWrenchVisual->GetName());

    this->dataPtr->applyWrenchVisual->Fini();
  }
  this->dataPtr->applyWrenchVisual.reset();

  this->deleteLater();
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::SetModel(const std::string &_modelName)
{
  if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
    return false;

  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(_modelName);

  if (!vis)
  {
    gzerr << "Model [" << _modelName << "] could not be found." << std::endl;
    return false;
  }

  this->dataPtr->modelName = _modelName;

  // Check if model/link hasn't been deleted on PreRender
  this->dataPtr->connections.push_back(
      event::Events::ConnectPreRender(
      boost::bind(&ApplyWrenchDialog::OnPreRender, this)));

  this->dataPtr->modelLabel->setText(("<b>Model:</b> " + _modelName).c_str());

  // Don't fire signals while inserting items
  this->dataPtr->linksComboBox->blockSignals(true);
  this->dataPtr->linksComboBox->clear();

  for (unsigned int i = 0; i < vis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = vis->GetChild(i);
    std::string linkName = childVis->GetName();

    // Issue #1553: This is failing to get real links sometimes:
    // uint32_t flags = childVis->GetVisibilityFlags();
    // if (!((flags != GZ_VISIBILITY_ALL) && (flags & GZ_VISIBILITY_GUI)))
    if (linkName.find("_GL_MANIP_") == std::string::npos)
    {
      std::string unscopedLinkName = linkName.substr(linkName.find("::") + 2);
      this->dataPtr->linksComboBox->addItem(
          QString::fromStdString(unscopedLinkName));

      // Get CoM from link's COMVisual
      for (unsigned int j = 0; j < childVis->GetChildCount(); ++j)
      {
        rendering::COMVisualPtr comVis =
            boost::dynamic_pointer_cast<rendering::COMVisual>(
            childVis->GetChild(j));

        if (comVis)
        {
          this->dataPtr->linkToCOMMap[linkName] = comVis->GetInertiaPose().pos;
          break;
        }
      }
    }
  }

  // Sort alphabetically
  QSortFilterProxyModel *proxy = new QSortFilterProxyModel(
      this->dataPtr->linksComboBox);
  proxy->setSourceModel(this->dataPtr->linksComboBox->model());
  this->dataPtr->linksComboBox->model()->setParent(proxy);
  this->dataPtr->linksComboBox->setModel(proxy);
  this->dataPtr->linksComboBox->model()->sort(0);

  this->dataPtr->linksComboBox->blockSignals(false);

  if (this->dataPtr->linksComboBox->count() > 0)
    return true;

  gzerr << "Couldn't find links in model ' [" << _modelName << "]."
      << std::endl;

  return false;
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::SetLink(const std::string &_linkName)
{
  if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
    return false;

  // Select on combo box
  std::string unscopedLinkName = _linkName.substr(_linkName.find("::") + 2);
  int index = -1;
  for (int i = 0; i < this->dataPtr->linksComboBox->count(); ++i)
  {
    if ((this->dataPtr->linksComboBox->itemText(i)).toStdString() ==
        unscopedLinkName)
    {
      index = i;
      break;
    }
  }
  if (index == -1)
  {
    gzerr << "Link [" << _linkName << "] could not be found in the combo box."
          << std::endl;
    return false;
  }
  this->dataPtr->linksComboBox->setCurrentIndex(index);

  // Visual
  this->dataPtr->linkName = _linkName;
  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->dataPtr->linkName);

  if (!vis)
  {
    gzerr << "A visual named [" << this->dataPtr->linkName
          << "] could not be found." << std::endl;
    return false;
  }
  this->dataPtr->linkVisual = vis;
  this->AttachVisuals();

  // Set publisher
  std::string topicName = "~/";
  topicName += this->dataPtr->linkName + "/wrench";
  boost::replace_all(topicName, "::", "/");

  this->dataPtr->wrenchPub.reset();
  this->dataPtr->wrenchPub =
      this->dataPtr->node->Advertise<msgs::Wrench>(topicName);

  // MouseRelease filter to gain focus
  if (this->dataPtr->applyWrenchVisual)
  {
    MouseEventHandler::Instance()->AddReleaseFilter(
        "dialog_"+this->dataPtr->applyWrenchVisual->GetName(),
        boost::bind(&ApplyWrenchDialog::OnMouseRelease, this, _1));
  }

  return true;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetLink(const QString _linkName)
{
  if (!this->SetLink(this->dataPtr->modelName + "::" + _linkName.toStdString()))
    this->Fini();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApplyAll()
{
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->dataPtr->forceVector);
  msgs::Set(msg.mutable_torque(), this->dataPtr->torqueVector);
  msgs::Set(msg.mutable_force_offset(), this->dataPtr->forcePosVector);

  this->dataPtr->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApplyForce()
{
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->dataPtr->forceVector);
  msgs::Set(msg.mutable_torque(), math::Vector3::Zero);
  msgs::Set(msg.mutable_force_offset(), this->dataPtr->forcePosVector);

  this->dataPtr->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApplyTorque()
{
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), math::Vector3::Zero);
  msgs::Set(msg.mutable_torque(), this->dataPtr->torqueVector);

  this->dataPtr->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnCancel()
{
  this->Fini();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForcePosChanged(double /*_value*/)
{
  // Update forcePos vector with values from XYZ spins
  this->SetForcePos(
      math::Vector3(this->dataPtr->forcePosXSpin->value(),
                    this->dataPtr->forcePosYSpin->value(),
                    this->dataPtr->forcePosZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceMagChanged(double /*_magnitude*/)
{
  // Update force vector proportionally
  // Normalize current vector
  math::Vector3 v = this->dataPtr->forceVector;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  this->SetForce(v * this->dataPtr->forceMagSpin->value());
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceChanged(double /*_value*/)
{
  // Update force vector with values from XYZ spins
  this->SetForce(math::Vector3(this->dataPtr->forceXSpin->value(),
                               this->dataPtr->forceYSpin->value(),
                               this->dataPtr->forceZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceClear()
{
  this->SetForce(math::Vector3::Zero);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueMagChanged(double /*_magnitude*/)
{
  // Update torque vector proportionally
  // Normalize current vector
  math::Vector3 v = this->dataPtr->torqueVector;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  this->SetTorque(v * this->dataPtr->torqueMagSpin->value());
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueChanged(double /*_value*/)
{
  // Update torque vector with values from XYZ spins
  this->SetTorque(math::Vector3(this->dataPtr->torqueXSpin->value(),
                               this->dataPtr->torqueYSpin->value(),
                               this->dataPtr->torqueZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueClear()
{
  this->SetTorque(math::Vector3::Zero);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleComRadio(bool _checked)
{
  if (_checked)
  {
    this->SetForcePos(this->dataPtr->comVector);
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetSpinValue(QDoubleSpinBox *_spin, double _value)
{
  _spin->blockSignals(true);
  _spin->setValue(_value);
  _spin->blockSignals(false);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetForcePos(const math::Vector3 &_forcePos)
{
  this->dataPtr->forcePosVector = _forcePos;

  // Spins
  this->SetSpinValue(this->dataPtr->forcePosXSpin, _forcePos.x);
  this->SetSpinValue(this->dataPtr->forcePosYSpin, _forcePos.y);
  this->SetSpinValue(this->dataPtr->forcePosZSpin, _forcePos.z);

  // Check COM box
  if (_forcePos == this->dataPtr->comVector)
  {
    this->dataPtr->comRadio->setChecked(true);
  }
  else
  {
    this->dataPtr->forcePosRadio->setChecked(true);
  }

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
  {
    gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
    return;
  }

  this->dataPtr->applyWrenchVisual->SetForcePos(this->dataPtr->forcePosVector);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetForce(const math::Vector3 &_force)
{
  this->dataPtr->forceVector = _force;

  // Spins
  this->SetSpinValue(this->dataPtr->forceXSpin, _force.x);
  this->SetSpinValue(this->dataPtr->forceYSpin, _force.y);
  this->SetSpinValue(this->dataPtr->forceZSpin, _force.z);
  this->SetSpinValue(this->dataPtr->forceMagSpin, _force.GetLength());

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
  {
    gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
    return;
  }

  this->dataPtr->applyWrenchVisual->SetForce(_force);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetTorque(const math::Vector3 &_torque)
{
  this->dataPtr->torqueVector = _torque;

  // Spins
  this->SetSpinValue(this->dataPtr->torqueXSpin, _torque.x);
  this->SetSpinValue(this->dataPtr->torqueYSpin, _torque.y);
  this->SetSpinValue(this->dataPtr->torqueZSpin, _torque.z);
  this->SetSpinValue(this->dataPtr->torqueMagSpin, _torque.GetLength());

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
  {
    gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
    return;
  }

  this->dataPtr->applyWrenchVisual->SetTorque(_torque);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetCoM(const math::Vector3 &_com)
{
  this->dataPtr->comVector = _com;

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
  {
    gzwarn << "No wrench visual found, so it won't be updated" << std::endl;
    return;
  }

  this->dataPtr->applyWrenchVisual->SetCoM(this->dataPtr->comVector);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnPreRender()
{
  if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
    return;

  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->dataPtr->linkName);

  // Close dialog in case visual has been deleted
  if (!vis)
    this->Fini();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::AttachVisuals()
{
  if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
  {
    gzerr << "Camera or scene missing" << std::endl;
    return;
  }
  if (!this->dataPtr->linkVisual)
  {
    gzerr << "No link visual specified." << std::endl;
    return;
  }

  // Attaching for the first time
  if (!this->dataPtr->applyWrenchVisual)
  {
    // Generate unique name
    std::string visNameBase = this->dataPtr->modelName + "__APPLY_WRENCH__";
    rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
        GetVisual(visNameBase);

    std::string visName(visNameBase);
    int count = 0;
    while (vis)
    {
      visName = visNameBase + std::to_string(count);
      vis = gui::get_active_camera()->GetScene()->GetVisual(visName);
      ++count;
    }

    this->dataPtr->applyWrenchVisual.reset(new rendering::ApplyWrenchVisual(
        visName, this->dataPtr->linkVisual));

    this->dataPtr->applyWrenchVisual->Load();
  }
  // Different link
  else if (!this->dataPtr->applyWrenchVisual->GetParent() ||
      this->dataPtr->applyWrenchVisual->GetParent() !=
      this->dataPtr->linkVisual)
  {
    this->dataPtr->linkVisual->AttachVisual(this->dataPtr->applyWrenchVisual);
    this->dataPtr->applyWrenchVisual->Resize();
  }

  if (!this->dataPtr->applyWrenchVisual)
  {
    gzwarn << "Failed to attach wrench visual. " <<
        "Dialog will work without it." << std::endl;
  }

  // Set COM
  this->SetCoM(this->dataPtr->linkToCOMMap[this->dataPtr->linkName]);
  // Apply force at com by default
  this->SetForcePos(this->dataPtr->comVector);
  this->SetTorque(this->dataPtr->torqueVector);
  this->SetForce(this->dataPtr->forceVector);
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::OnMouseRelease(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera || !this->dataPtr->applyWrenchVisual)
    return false;

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos);

  if (!vis || _event.dragging)
    return false;

  // Force/torque clicked: activate dialog and prevent event propagation
  if (vis == this->dataPtr->applyWrenchVisual->GetForceVisual() ||
      vis == this->dataPtr->applyWrenchVisual->GetTorqueVisual())
  {
    this->ActivateWindow();
    return true;
  }
  // Link clicked: activate dialog and propagate event
  else if (vis->GetNthAncestor(2) ==
      this->dataPtr->applyWrenchVisual->GetNthAncestor(2))
  {
    this->ActivateWindow();
  }
  return false;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ActivateWindow()
{
  if (!this->isActiveWindow())
  {
    // Clear focus before activating not to trigger FucusIn
    QWidget *focusedWidget = this->focusWidget();
    if (focusedWidget)
      focusedWidget->clearFocus();

    this->activateWindow();
  }
}
