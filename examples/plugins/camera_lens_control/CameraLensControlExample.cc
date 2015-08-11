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
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "CameraLensControlExample.hh"

#include "ImageFrame.hh"

#include <QtUiTools/QUiLoader>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CameraLensControlExample)

/////////////////////////////////////////////////
CameraLensControlExample::CameraLensControlExample()
  : GUIPlugin()
{
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Open file with GUI description
  QFile file("./mainwindow.ui");
  file.open(QFile::ReadOnly);

  // Load GUI from file
  QUiLoader loader;
  QWidget *contentWidget = loader.load(&file,this);
  file.close();

  // add loaded widget to layout
  frameLayout->addWidget(contentWidget);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(342, 300);

  // Load child widgets and associated events
  this->LoadGUIComponents(contentWidget);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Camera factory
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

  // Listen for selected element in gazebo
  this->selectionSub = this->node->Subscribe("~/selection",&CameraLensControlExample::OnSelect,this);

  this->imageSub.reset();
}

/////////////////////////////////////////////////
CameraLensControlExample::~CameraLensControlExample()
{
  // delete this->imgFrame;
}

/////////////////////////////////////////////////
void CameraLensControlExample::LoadGUIComponents(QWidget *_parent)
{
  GZ_ASSERT(_parent != NULL, "Invalid widget loaded");

  gzmsg << "OK\n";
  this->pbSpawn     = _parent->findChild<QPushButton*>("pbSpawn");
  this->pbCalibrate = _parent->findChild<QPushButton*>("pbCalibrate");
  this->lbName      = _parent->findChild<QLabel*>("lbName");
  this->cbType      = _parent->findChild<QComboBox*>("cbType");
  this->cbFun       = _parent->findChild<QComboBox*>("cbFun");
  this->sbC1        = _parent->findChild<QDoubleSpinBox*>("sbC1");
  this->sbC2        = _parent->findChild<QDoubleSpinBox*>("sbC2");
  this->sbC3        = _parent->findChild<QDoubleSpinBox*>("sbC3");
  this->sbF         = _parent->findChild<QDoubleSpinBox*>("sbF");
  this->sbCA        = _parent->findChild<QDoubleSpinBox*>("sbCA");
  this->cbCircular  = _parent->findChild<QCheckBox*>("cbCircular");

  QWidget *wImagePlace = _parent->findChild<QWidget*>("wImagePlace");

  // this->imgFrame = new gui::ImageFrame(wImagePlace);
  // this->imgFrame->setMinimumSize(201,201);
  // this->imgFrame->setMaximumSize(201,201);
  // this->imgFrame->setVisible(true);

  // fill combo boxes with values
  this->cbType->addItem("gnomonical");
  this->cbType->addItem("stereographic");
  this->cbType->addItem("stereographic_");
  this->cbType->addItem("equidistant");
  this->cbType->addItem("equisolid_angle");
  this->cbType->addItem("orthographic");
  this->cbType->addItem("custom");

  this->cbFun->addItem("sin");
  this->cbFun->addItem("tan");
  this->cbFun->addItem("id");

  this->pbCalibrate->setVisible(false);

  // bind Events
  connect(this->cbType,SIGNAL(currentIndexChanged(int)),this,SLOT(OnValueChanged()));
  connect(this->cbFun,SIGNAL(currentIndexChanged(int)),this,SLOT(OnValueChanged()));
  connect(this->cbCircular,SIGNAL(stateChanged(int)),this,SLOT(OnValueChanged()));

  connect(this->sbC1,SIGNAL(valueChanged(double)),this,SLOT(OnValueChanged()));
  connect(this->sbC2,SIGNAL(valueChanged(double)),this,SLOT(OnValueChanged()));
  connect(this->sbC3,SIGNAL(valueChanged(double)),this,SLOT(OnValueChanged()));
  connect(this->sbF,SIGNAL(valueChanged(double)),this,SLOT(OnValueChanged()));
  connect(this->sbCA,SIGNAL(valueChanged(double)),this,SLOT(OnValueChanged()));

  // Add listener for spawn button
  connect(pbSpawn, SIGNAL(clicked()), this, SLOT(OnButtonSpawn()));
}

/////////////////////////////////////////////////
void CameraLensControlExample::OnButtonSpawn()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='wideanglecamera_spawn_" << this->counter++ << "'>"
    << "  <pose>0 0 1.5 0 0 0</pose>"
    << "    <link name='link'>"
    << "      <inertial>"
    << "        <mass>0.1</mass>"
    << "      </inertial>"
    << "      <collision name='collision'>"
    << "        <geometry>"
    << "          <box>"
    << "            <size>0.1 0.1 0.1</size>"
    << "          </box>"
    << "        </geometry>"
    << "      </collision>"
    << "      <visual name='visual'>"
    << "        <geometry>"
    << "          <box>"
    << "            <size>0.1 0.1 0.1</size>"
    << "          </box>"
    << "        </geometry>"
    << "      </visual>"
    << "      <sensor name='camera' type='wideanglecamera'>"
    << "        <camera>"
    << "          <horizontal_fov>1.047</horizontal_fov>"
    << "          <save enabled='false'><path>/tmp/imgcache/</path></save>"
    << "          <image>"
    << "            <width>400</width>"
    << "            <height>400</height>"
    << "          </image>"
    << "          <clip>"
    << "            <near>0.1</near>"
    << "            <far>100</far>"
    << "          </clip>"
    << "          <lens>"
    << "            <type>equisolid_angle</type>"
    << "            <circular>true</circular>"
    << "            <advertise>true</advertise>"
    << "            <custom_function>"
    << "              <c1>2</c1>"
    << "              <c2>2.0</c2>"
    << "              <f>0.25</f>"
    << "              <fun>tan</fun>"
    << "            </custom_function>"
    << "            <env_texture_size>512</env_texture_size>"
    << "            <cutoff_angle>1.5707</cutoff_angle>"   
    << "          </lens>"
    << "        </camera>"
    << "        <always_on>1</always_on>"
    << "        <update_rate>30</update_rate>"
    << "        <visualize>true</visualize>"
    << "      </sensor>"
    << "    </link>"
    << "  </model>"
    << "</sdf>";

// Send the model to the gazebo server
  msgs::Factory msg;
  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);
}

/////////////////////////////////////////////////
void CameraLensControlExample::OnSelect(ConstSelectionPtr &_msg)
{
  lbName->setText(QString::fromStdString(_msg->name()));

  gzmsg << "selected: " << _msg->name();
  this->selectedElementName = _msg->name();

  std::string t("wideanglecamera");
  if(_msg->name().compare(0,t.length(),t) == 0)
  {
    // Do the thing
    gzmsg << " [Appropriate element]";

    if(this->imageSub)
      this->imageSub->Unsubscribe();

    // this->imageSub = this->node->Subscribe("~/"+_msg->name()+"/link/camera/image",
    //     &CameraLensControlExample::OnImageUpdate,this);

    // subscribe for the info messages
    this->infoSub = this->node->Subscribe("~/"+_msg->name()+"/link/camera/lens_info",
        &CameraLensControlExample::OnCameraProjectionCmd,this);

    this->cameraControlPub = this->node->Advertise<msgs::CameraLensCmd>(
      "~/"+_msg->name()+"/link/camera/lens_control",10);

    this->ignoreInfoMessages = false;
  }

  gzmsg << std::endl;
}

/////////////////////////////////////////////////
void CameraLensControlExample::OnImageUpdate(ConstImageStampedPtr &_msg)
{
  // Display received image
  // this->imgFrame->OnImage(_msg->image());
}

/////////////////////////////////////////////////
void CameraLensControlExample::OnCameraProjectionCmd(ConstCameraLensCmdPtr &_msg)
{
  if(this->ignoreInfoMessages)
    return;
  else
    this->ignoreInfoMessages = true;

  if(_msg->has_c1())
    this->sbC1->setValue(_msg->c1());
  if(_msg->has_c2())
    this->sbC2->setValue(_msg->c2());
  if(_msg->has_c3())
    this->sbC3->setValue(_msg->c3());
  if(_msg->has_f())
    this->sbF->setValue(_msg->f());

  this->sbCA->setValue(_msg->cutoff_angle());

  this->cbType->setCurrentIndex(this->cbType->findText(QString::fromStdString(_msg->type())));

  if(_msg->has_fun())
    this->cbFun->setCurrentIndex(this->cbFun->findText(QString::fromStdString(_msg->fun())));

  if(_msg->has_circular())
    this->cbCircular->setChecked(_msg->circular());

  // Information got, runtime update is not needed
  // this->infoSub->Unsubscribe();
}

/////////////////////////////////////////////////
void CameraLensControlExample::OnValueChanged()
{
  if(this->selectedElementName != "")
  {
    msgs::CameraLensCmd msg;

    msg.set_name(this->selectedElementName);

    msg.set_destiny(msgs::CameraLensCmd_CmdDestiny_SET);

    msg.set_type(this->cbType->currentText().toUtf8().constData());

    if(this->cbType->currentText() == "custom")
    {
      msg.set_fun(this->cbFun->currentText().toUtf8().constData());

      msg.set_c1(this->sbC1->value());
      msg.set_c2(this->sbC2->value());
      msg.set_c3(this->sbC3->value());
      msg.set_f(this->sbF->value());
    }

    msg.set_cutoff_angle(this->sbCA->value());

    msg.set_circular(this->cbCircular->isChecked());

    this->cameraControlPub->Publish(msg);
    gzmsg << "Control message sent" << std::endl;
  }
}