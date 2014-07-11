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

#include "gazebo/gui/OculusConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
OculusConfig::OculusConfig(QWidget *_parent)
  : QDialog(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("oculusconfig");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Oculus Rift configuration"));

  // Create the main layout for this widget.
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;

  QGridLayout *contentLayout = new QGridLayout;

  // Step 1.
  // QHBoxLayout *step1Layout = new QHBoxLayout;
  QVBoxLayout *step1LeftLayout = new QVBoxLayout;
  const QString Step1Title = "1. Position headset as display";
  const QString Step1Text = "Tell us how wide your monitor is so we can"
                            " add the\nOculus next to it.";
  const QString Step1Res  = "(your monitor's resolution width)";

  QFont bigFont;
  bigFont.setPointSize(13);
  QLabel *step1Title = new QLabel(Step1Title);
  step1Title->setFont(bigFont);
  QLabel *step1Text = new QLabel(Step1Text);

  QLabel *image1 = new QLabel;
  QPixmap pixmap1(":/images/help_icon.png");
  image1->setPixmap(pixmap1.scaled(25, 25, Qt::KeepAspectRatio));
  image1->setToolTip("<html><img src=:/images/oculus_position.png/></html>");

  QHBoxLayout *step1TitleLayout = new QHBoxLayout;

  step1TitleLayout->addWidget(step1Title);
  step1TitleLayout->addWidget(image1);


  step1LeftLayout->addWidget(step1Text);
  step1LeftLayout->addLayout(step1TitleLayout);

  QGridLayout *step1GridLayout = new QGridLayout;
  step1GridLayout->setContentsMargins(0, 20, 11, 11);
  QLabel *xLabel = new QLabel("x:");
  xEdit = new QLineEdit();
  xEdit->setFixedWidth(45);
  xEdit->setMaxLength(4);
  xEdit->setValidator( new QIntValidator(0, 9999, this));
  QLabel *xResHelpLabel = new QLabel(Step1Res);
  QLabel *yLabel = new QLabel("y:");
  yEdit = new QLineEdit("   0");
  yEdit->setFixedWidth(45);
  yEdit->setMaxLength(4);
  yEdit->setValidator( new QIntValidator(0, 9999, this));

  // Insert all the widgets into the grid layout.
  step1GridLayout->addWidget(xLabel, 0, 1, Qt::AlignLeft);
  step1GridLayout->addWidget(xEdit, 0, 2);
  step1GridLayout->addWidget(xResHelpLabel, 0, 3);
  step1GridLayout->addWidget(yLabel, 1, 1, Qt::AlignLeft);
  step1GridLayout->addWidget(yEdit, 1, 2);

  step1LeftLayout->addLayout(step1GridLayout);

  // Step 1 right side.
  /*QVBoxLayout *step1RightLayout = new QVBoxLayout;
  QLabel *image1 = new QLabel;
  QPixmap pixmap1(":/images/help_icon.png");
  image1->setPixmap(pixmap1.scaled(35, 35, Qt::KeepAspectRatio));

  step1RightLayout->addWidget(image1);*/

  // Add left and right to the step1 layout.
  //step1Layout->addLayout(step1LeftLayout);
  //step1RightLayout->setContentsMargins(200, 0, 0, 0);
  //step1Layout->addLayout(step1RightLayout);

  // Add the step1 layout to the main layout.
  contentLayout->addLayout(step1LeftLayout, 0, 1);
  //contentLayout->addLayout(step1RightLayout, 0, 2, Qt::AlignRight);

  // Step 2.
  //QHBoxLayout *step2Layout = new QHBoxLayout;
  QVBoxLayout *step2LeftLayout = new QVBoxLayout;
  const QString Step2Title = "2. Attach Oculus to visual";
  const QString Step2Text = "Select the visual to which you want to attach "
                            "the\nOculus. This will determine the device's "
                            "point\n of view.";
  const QString Step2Help = "To view only the visuals for a specific model,"
                            " close this window,\nright-click on the desired"
                            " model, and select 'Attach Oculus Rift'.";
  QLabel *step2Title = new QLabel(Step2Title);
  step2Title->setFont(bigFont);
  QLabel *step2Text = new QLabel(Step2Text);
  // Create the widget for reading the visual link where the Oculus will
  // be attached.
  // QLabel *visualLabel = new QLabel("Visual:");
  visualEdit = new QLineEdit();
  visualEdit->setMaximumWidth(350);
  visualEdit->setFixedWidth(400);
  visualEdit->setContentsMargins(0, 20, 11, 11);
  QLabel *step2Help = new QLabel(Step2Help);
  QFont smallFont;
  smallFont.setPointSize(9);
  step2Help->setFont(smallFont);
  step2Help->setContentsMargins(0, -2, 0, 0);

  step2LeftLayout->addWidget(step2Title);
  step2LeftLayout->addWidget(step2Text);
  step2LeftLayout->addWidget(visualEdit);
  step2LeftLayout->addWidget(step2Help);
  step2LeftLayout->setContentsMargins(0, 35, 0, 0);

  // Step 2 right side.
  QVBoxLayout *step2RightLayout = new QVBoxLayout;
  QLabel *image2 = new QLabel;
  QPixmap pixmap2(":/images/oculus_visual.png");
  image2->setPixmap(pixmap2.scaled(150, 150, Qt::KeepAspectRatio));

  step2RightLayout->addWidget(image2);

  // Add left and right to the step2 layout.
  /*step2Layout->addLayout(step2LeftLayout, Qt::AlignLeft);
  step2RightLayout->setContentsMargins(200, 0, 0, 0);
  step2Layout->addLayout(step2RightLayout, Qt::AlignRight);

  // Add the step2 layout to the main layout.
  contentLayout->addLayout(step2Layout);*/
  step2LeftLayout->setContentsMargins(0, 50, 0, 0);
  step2RightLayout->setContentsMargins(70, 50, 0, 0);

  contentLayout->addLayout(step2LeftLayout, 1, 1);
  //contentLayout->addLayout(step2RightLayout, 1, 2);

  // Step 3.
  //QHBoxLayout *step3Layout = new QHBoxLayout;
  QVBoxLayout *step3LeftLayout = new QVBoxLayout;
  const QString Step3Title = "3. Enter offsets (optional)";
  const QString Step3Text = "By default, the Oculus will be aligned with the\n"
                            "origin of the visual. You may add offsets below\n"
                            "to change the position and orientation";

  QLabel *step3Title = new QLabel(Step3Title);
  step3Title->setFont(bigFont);
  QLabel *step3Text = new QLabel(Step3Text);
  step3LeftLayout->addWidget(step3Title);
  step3LeftLayout->addWidget(step3Text);

  // Create the widgets for reading the offset applied to the Oculus.
  QGridLayout *step3GridLayout = new QGridLayout;
  step3GridLayout->setContentsMargins(0, 20, 11, 11);
  QFont underlineFont;
  underlineFont.setUnderline(true);
  QLabel *positionLabel = new QLabel("Position");
  positionLabel->setFont(underlineFont);
  QLabel *xOffsetLabel = new QLabel("x:");
  xOffsetEdit = new QLineEdit();
  xOffsetEdit->setFixedWidth(45);
  xOffsetEdit->setMaxLength(10);
  xOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *yOffsetLabel = new QLabel("y:");
  yOffsetEdit = new QLineEdit();
  yOffsetEdit->setFixedWidth(45);
  yOffsetEdit->setMaxLength(10);
  yOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *zOffsetLabel = new QLabel("z:");
  zOffsetEdit = new QLineEdit();
  zOffsetEdit->setFixedWidth(45);
  zOffsetEdit->setMaxLength(10);
  zOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *orientationLabel = new QLabel("Orientation");
  orientationLabel->setFont(underlineFont);
  QLabel *rollOffsetLabel = new QLabel("roll:");
  rollOffsetEdit = new QLineEdit();
  rollOffsetEdit->setFixedWidth(45);
  rollOffsetEdit->setMaxLength(10);
  rollOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *pitchOffsetLabel = new QLabel("pitch:");
  pitchOffsetEdit = new QLineEdit();
  pitchOffsetEdit->setFixedWidth(45);
  pitchOffsetEdit->setMaxLength(10);
  pitchOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *yawOffsetLabel = new QLabel("yaw:");
  yawOffsetEdit = new QLineEdit();
  yawOffsetEdit->setFixedWidth(45);
  yawOffsetEdit->setMaxLength(10);
  yawOffsetEdit->setValidator( new QDoubleValidator());

  // Insert all the widgets into the grid layout.
  step3GridLayout->addWidget(positionLabel, 0, 1, Qt::AlignLeft);
  step3GridLayout->addWidget(xOffsetLabel, 0, 2);
  step3GridLayout->addWidget(xOffsetEdit, 0, 3);
  step3GridLayout->addWidget(yOffsetLabel, 0, 4);
  step3GridLayout->addWidget(yOffsetEdit, 0, 5);
  step3GridLayout->addWidget(zOffsetLabel, 0, 6);
  step3GridLayout->addWidget(zOffsetEdit, 0, 7);
  step3GridLayout->addWidget(orientationLabel, 1, 1, Qt::AlignLeft);
  step3GridLayout->addWidget(rollOffsetLabel, 1, 2, Qt::AlignRight);
  step3GridLayout->addWidget(rollOffsetEdit, 1, 3);
  step3GridLayout->addWidget(pitchOffsetLabel, 1, 4);
  step3GridLayout->addWidget(pitchOffsetEdit, 1, 5);
  step3GridLayout->addWidget(yawOffsetLabel, 1, 6);
  step3GridLayout->addWidget(yawOffsetEdit, 1, 7);
  step3LeftLayout->setContentsMargins(0, 35, 0, 0);
  step3LeftLayout->addLayout(step3GridLayout);

  // Step 3 right side.
  QVBoxLayout *step3RightLayout = new QVBoxLayout;
  QLabel *image3 = new QLabel;
  QPixmap pixmap3(":/images/oculus_offsets.png");
  image3->setPixmap(pixmap3.scaled(300, 300, Qt::KeepAspectRatio));

  step3RightLayout->addWidget(image3);

  // Add left and right to the step3 layout.
  /*step3Layout->addLayout(step3LeftLayout);
  step3Layout->addLayout(step3RightLayout);

  // Add the step3 layout to the main layout.
  contentLayout->addLayout(step3Layout);*/

  step3LeftLayout->setContentsMargins(0, 50, 0, 0);
  step3RightLayout->setContentsMargins(0, 50, 0, 0);


  contentLayout->addLayout(step3LeftLayout, 2, 1);
  //contentLayout->addLayout(step3RightLayout, 2, 2);

  // Create the remaining elements.
  frame->setLayout(contentLayout);

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton("Cancel");
  connect(cancelButton, SIGNAL(clicked()),
          this, SLOT(OnCancel()));

  this->okayButton = new QPushButton("Okay");
  //this->okayButton->setEnabled(false);
  connect(this->okayButton, SIGNAL(clicked()),
          this, SLOT(OnOkay()));

  buttonLayout->addWidget(cancelButton);
  buttonLayout->addStretch(2);
  buttonLayout->addWidget(this->okayButton);

  mainLayout->addWidget(frame);
  mainLayout->addLayout(buttonLayout);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
OculusConfig::~OculusConfig()
{

}

/////////////////////////////////////////////////
void OculusConfig::Update()
{
  this->oculusWindowCoordinates.Set(
    this->xEdit->text().toDouble(),
    this->yEdit->text().toDouble());

  this->visual = this->visualEdit->text().toUtf8().constData();

  math::Vector3 offsetPos = math::Vector3(
    this->xOffsetEdit->text().toDouble(),
    this->yOffsetEdit->text().toDouble(),
    this->zOffsetEdit->text().toDouble());

  math::Quaternion offsetRot = math::Quaternion(
    this->rollOffsetEdit->text().toDouble(),
    this->pitchOffsetEdit->text().toDouble(),
    this->yawOffsetEdit->text().toDouble());

  this->offset.Set(offsetPos, offsetRot);
}

/////////////////////////////////////////////////
void OculusConfig::OnOkay()
{
  this->done(QDialog::Accepted);
  this->Update();
}

/////////////////////////////////////////////////
void OculusConfig::OnCancel()
{
  this->done(QDialog::Rejected);
}

/////////////////////////////////////////////////
math::Vector2d OculusConfig::GetOculuswindowCoordinates()
{
  return this->oculusWindowCoordinates;
}

/////////////////////////////////////////////////
std::string OculusConfig::GetVisual()
{
  return this->visual;
}

/////////////////////////////////////////////////
math::Pose OculusConfig::GetOffset()
{
  return this->offset;
}
