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
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/GuiEvents.hh>

#include "TaskButton.hh"
#include "HaptixGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(HaptixGUIPlugin)

/////////////////////////////////////////////////
HaptixGUIPlugin::HaptixGUIPlugin()
  : GUIPlugin()
{
  // Read parameters
  std::string handImgFilename = common::SystemPaths::Instance()->FindFileURI
                                  ("file://media/materials/textures/hand.svg");

  // Parameters for sensor contact visualization
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {background-color: rgba(255, 255, 255, 255);"
      "color: rgba(100, 100, 100, 255);"
      "}"
      );

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->setContentsMargins(4, 4, 0, 0);
  mainFrame->setLayout(frameLayout);

  this->setPalette(QPalette(QColor(255, 255, 255, 0)));

  // Create the hand
    // Create a QGraphicsView to draw the finger force contacts
    this->handScene = new QGraphicsScene(QRectF(0, 0, 400, 300));
    QGraphicsView *handView = new QGraphicsView(this->handScene);
    handView->setStyleSheet("border: 0px");
    handView->setSizePolicy(QSizePolicy::Expanding,
                            QSizePolicy::MinimumExpanding);

    // Load the hand image
    QPixmap handImg = QPixmap(QString(handImgFilename.c_str()));
    QGraphicsPixmapItem *handItem = new QGraphicsPixmapItem(handImg);
    handItem->setPos(-20, -73);

    // Draw the hand on the canvas
    this->handScene->addItem(handItem);

  // Create the task layout
    this->taskTab = new QTabWidget();
    this->taskTab->setStyleSheet(
        "QTabWidget {"
          "border: 1px solid rgba(128, 128, 128, 255)"
        "}"

        "QTabWidget::pane {"
          "top: -1px;"
          "background-color: #ff00ff;"
          "border: 1px solid rgba(128, 128, 128, 255);"
        "}"

        "QTabBar::tab-bar {"
          "left: 5px"
        "}"

        "QTabBar::tab {"
          "color: rgba(100, 100, 100, 255);"
          "border: 1px solid rgba(128, 128, 128, 255);"
          "padding: 0px;"
          "border-top-left-radius: 4px;"
          "border-top-right-radius: 4px;"
          "background-color: rgba(200, 200, 200, 255);"
        "}"

        "QTabBar::tab:selected {"
          "color: rgba(100, 100, 100, 255);"
          "background-color: rgba(255, 255, 255, 255);"
          "border: 1px solid rgba(128, 128, 128, 255);"
          "border-bottom: 1px solid rgba(255, 255, 255, 255);"
        "}"
        );

    QFrame *tabFrame = new QFrame();
    tabFrame->setContentsMargins(4, 0, 4, 0);
    QVBoxLayout *tabFrameLayout = new QVBoxLayout();
    tabFrame->setLayout(tabFrameLayout);

    this->instructionsView = new QTextEdit();
    this->instructionsView->setReadOnly(true);
    this->instructionsView->setMaximumHeight(60);
    this->instructionsView->setStyleSheet(
        "margin-top: 0px;"
        "margin-bottom: 0px;"
        "margin-left: 20px;"
        "margin-right: 20px;"
        "background-color: #ffffff"
        );

    tabFrameLayout->addWidget(taskTab);
    tabFrameLayout->addWidget(this->instructionsView);


  QHBoxLayout *cycleButtonLayout = new QHBoxLayout();
  QPushButton *resetButton = new QPushButton();
  resetButton->setText(QString("Reset Test"));
  resetButton->setStyleSheet(
      "background-color: rgba(120, 120, 120, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
  resetButton->setMaximumWidth(120);

  QPushButton *nextButton = new QPushButton();
  nextButton->setText(QString("Next Test"));
  nextButton->setStyleSheet(
      "background-color: rgba(23, 85, 138, 255);"
      "border: 0px;"
      "border-radius: 4px;"
      "color: #ffffff");
  connect(nextButton, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
  nextButton->setMaximumWidth(120);

  cycleButtonLayout->addWidget(resetButton);
  cycleButtonLayout->addWidget(nextButton);

  QFrame *cycleButtonFrame = new QFrame;
  cycleButtonFrame->setLayout(cycleButtonLayout);

  // Add all widgets to the main frame layout
  frameLayout->addWidget(handView, 1.0);
  frameLayout->addWidget(tabFrame);
  frameLayout->addWidget(instructionsView);
  frameLayout->addWidget(cycleButtonFrame);

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->move(10, 10);
  this->resize(450, 800);

  // Create a QueuedConnection to set contact visualization value.
  connect(this, SIGNAL(SetContactForce(QString, double)),
          this, SLOT(OnSetContactForce(QString, double)), Qt::QueuedConnection);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Create the publisher that communicates with the arrange plugin
  this->taskPub = this->node->Advertise<msgs::GzString>("~/arrange");

  // Connect to the PreRender Gazebo signal
  this->connections.push_back(event::Events::ConnectPreRender(
                              boost::bind(&HaptixGUIPlugin::PreRender, this)));

  this->currentTaskId = 0;

  /*

  this->ignNode = new ignition::transport::Node("haptix");
  ignNode->Advertise("arm_pose_inc");

  // Parse SDF to get the arm teleop commands
  sdf::ElementPtr command = elem->GetElement("commands");
  command = command->GetElement("command");
  while(command){
    char button;
    std::string name;
    float increment;
    command->GetAttribute("button")->Get(button);
    command->GetAttribute("name")->Get(name);
    command->GetAttribute("increment")->Get(increment);
    if(name.substr(0, 5).compare("motor") == 0){
      this->handCommands[button] = KeyCommand(button, name, increment);
    } else if(name.substr(0, 3).compare("arm") == 0){
      this->armCommands[button] = KeyCommand(button, name, increment);
    }
    buttonNames[name].push_back(button);

    command = command->GetNextElement();
  }

  sdf::ElementPtr index = elem->GetElement("indices");
  index = index->GetElement("index");
  while(index){
    std::string name;
    int num;
    index->GetAttribute("name")->Get(name);
    index->GetAttribute("num")->Get(num);
    std::vector<char> buttons = buttonNames[name];
    for(unsigned int i = 0; i < buttons.size(); i++){
      char button = buttons[i];
      if(this->handCommands.find(button) != this->handCommands.end()){
        this->handCommands[button].index = num;
      } else if (this->armCommands.find(button) != this->armCommands.end()){
        this->armCommands[button].index = num;
      }
    }
    index = index->GetNextElement();
  }

  // Set up hx objects
  //if (hx_getdeviceinfo(hxGAZEBO, &handDeviceInfo) != hxOK)
  //{
  //  std::cout << "hx_getdeviceinfo(): Request error. Cannot control hand."
  //            << std::endl;
  //}
  //for(int i = 0; i < handDeviceInfo.nmotor; i++)
  //{
  //  handCommand.ref_pos[i] = 0;
  //}

  //if(hx_update(hxGAZEBO, &handCommand, &handSensor) != hxOK ){
  //  std::cout << "hx_update(): Request error.\n" << std::endl;
  //}

  // Set up an array of subscribers for each contact sensor
  for(std::map<std::string, math::Vector2d>::iterator it =
        this->contactPoints.begin(); it != this->contactPoints.end(); it++)
  {
  }

  gui::KeyEventHandler::Instance()->SetAutoRepeat(true);
  gui::KeyEventHandler::Instance()->AddPressFilter("arat_gui",
                          boost::bind(&HaptixGUIPlugin::OnKeyPress, this, _1));
  */
}


/////////////////////////////////////////////////
HaptixGUIPlugin::~HaptixGUIPlugin()
{
  /*

  for (std::map<std::string, QGraphicsEllipseItem*>::iterator it =
       contactGraphicsItems.begin(); it != contactGraphicsItems.end(); it++)
  {
    delete it->second;
    it->second = NULL;
  }

  delete ignNode;
  ignNode = NULL;
  */
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::Load(sdf::ElementPtr _elem)
{
  this->circleSize = _elem->Get<int>("circle_size");

  this->forceMin = _elem->Get<double>("force_min");
  this->forceMax = _elem->Get<double>("force_max");

  this->colorMin = _elem->Get<common::Color>("color_min");
  this->colorMax = _elem->Get<common::Color>("color_max");

  this->handSide = _elem->Get<std::string>("hand_side");

  // Get contact names
  if (_elem->HasElement("contacts"))
  {
    sdf::ElementPtr contact = _elem->GetElement("contacts");
    contact = contact->GetElement("contact");

    while (contact)
    {
      if (contact->HasAttribute("name"))
      {
        // Read the contact data from SDF
        std::string contactName = contact->Get<std::string>("name");
        math::Vector2d contactPos = contact->Get<math::Vector2d>("pos");
        std::string topic = contact->Get<std::string>("topic");

        // Create a subscriber that receive contact data
        transport::SubscriberPtr sub = this->node->Subscribe(topic,
            &HaptixGUIPlugin::OnFingerContact, this);
        this->contactSubscribers.push_back(sub);

        this->contactGraphicsItems[contactName] =
          new QGraphicsEllipseItem(contactPos.x,
              contactPos.y, this->circleSize, this->circleSize);
        this->handScene->addItem(this->contactGraphicsItems[contactName]);

        this->contactGraphicsItems[contactName]->setBrush(
            QBrush(QColor(255, 255, 255, 0)));
        this->contactGraphicsItems[contactName]->setPen(
            QPen(QColor(153, 153, 153, 255)));

        // Get the position of the contact
        this->contactPoints[contactName] = contactPos;

        contact = contact->GetNextElement();
      }
    }
  }

  // Draw contact pads and force gauge
  {
    float scaleXPos = 365;
    float scaleWidth = 20;

    QGraphicsRectItem *forceScaleItem =
      new QGraphicsRectItem(scaleXPos, -40, scaleWidth, 400);
    forceScaleItem->setPen(QPen(QColor(255, 255, 255, 0)));
    QLinearGradient grad(0, 0, 0, 400);
    grad.setColorAt(0, QColor(255, 227, 32, 255));
    grad.setColorAt(1, QColor(255, 102, 102, 255));
    forceScaleItem->setBrush(grad);
    this->handScene->addItem(forceScaleItem);

    // Draw the lines and force values.
    int lineStep = 40;
    double force = this->forceMax;
    int steps = (440/40) - 1;
    double forceStep = (this->forceMax - this->forceMin)/steps;

    for (int i = -40; i < 400-lineStep; i += lineStep)
    {
      QGraphicsLineItem *lineItem =
        new QGraphicsLineItem(scaleXPos, i, scaleXPos + scaleWidth, i);
      lineItem->setPen(QPen(
            QBrush(QColor(255, 255, 255, 255)), 2.0));
      this->handScene->addItem(lineItem);

      std::stringstream forceStream;
      forceStream << std::fixed << std::setprecision(1) << force;
      force -= forceStep;

      QGraphicsTextItem *text = new QGraphicsTextItem(
          forceStream.str().c_str());
      text->setPos(scaleXPos + scaleWidth + 4, i-11.5);
      this->handScene->addItem(text);
    }

    // Draw the PSI label
    QGraphicsTextItem *psiText = new QGraphicsTextItem(tr("PSI"));
    psiText->setPos(scaleXPos-4, 362);
    this->handScene->addItem(psiText);
  }

  this->InitializeTaskView(_elem);


  /*
  elem->GetElement("scaleFactor")->GetValue()->Get(this->GUIScaleFactor);

  std::string buttonStyle;
  elem->GetElement("startButtonStyle")->GetValue()->Get(buttonStyle);
  this->startButtonStyle = QString(buttonStyle.c_str());
  elem->GetElement("stopButtonStyle")->GetValue()->Get(buttonStyle);
  this->stopButtonStyle = QString(buttonStyle.c_str());

  math::Vector2d handImgDims;
  elem->GetElement("handImgDimensions")->GetValue()->Get(handImgDims);
  this->handImgX = GUIScaleFactor*handImgDims[0];
  this->handImgY = GUIScaleFactor*handImgDims[1];

  elem->GetElement("iconDimensions")->GetValue()->Get(this->iconSize);
  this->iconSize *= GUIScaleFactor;
  this->circleSize *= GUIScaleFactor;
  */
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnFingerContact(ConstContactsPtr &_msg)
{
  // Parse out the finger name
  if (_msg->contact_size() > 0)
  {
    std::string collisionName1 = _msg->contact(0).collision1();
    std::string collisionName2 = _msg->contact(0).collision2();

    // Calculate the force
    msgs::Vector3d forceVector = _msg->contact(0).wrench(0).
      body_1_wrench().force();
    double force = math::Vector3(forceVector.x(), forceVector.y(),
        forceVector.z()).GetLength();

    if (this->contactPoints.find(collisionName1) != this->contactPoints.end())
    {
      // Draw the new force value
      this->SetContactForce(QString::fromStdString(collisionName1), force);
    }
    else if (this->contactPoints.find(collisionName2) !=
             this->contactPoints.end())
    {
      // Draw the new force value
      this->SetContactForce(QString::fromStdString(collisionName2), force);
    }
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnSetContactForce(QString _contactName, double _value)
{
  double colorArray[3];
  float forceRange = this->forceMax - this->forceMin;

  for (int i = 0; i < 3; ++i)
  {
    float colorRange = this->colorMax[i] - this->colorMin[i];
    colorArray[i] = colorRange/forceRange * _value + this->colorMin[i];

    if (colorArray[i] > this->colorMin[i])
      colorArray[i] = this->colorMin[i];
    else if (colorArray[i] < this->colorMax[i])
      colorArray[i] = this->colorMax[i];
  }

  QBrush color(QColor(colorArray[0], colorArray[1], colorArray[2]));

  this->contactGraphicsItems[_contactName.toStdString()]->setBrush(color);
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::PreRender()
{
  // Hide the scene tree.
  gui::Events::sceneTreeVisibility(false);

  // Fade out old force values
  for (std::map<std::string, QGraphicsEllipseItem*>::iterator iter =
      this->contactGraphicsItems.begin();
      iter != this->contactGraphicsItems.end(); ++iter)
  {
    QBrush brush = iter->second->brush();
    QColor color = brush.color();
    color.setAlpha(std::max(0, color.alpha()-5));
    brush.setColor(color);

    iter->second->setBrush(brush);
  }
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::mousePressEvent(QMouseEvent *_mouseEvent)
{
  printf("On Mouse Press\n");
  std::cout << "Pos[" << _mouseEvent->x() << " " << _mouseEvent->y() << "]\n";
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::InitializeTaskView(sdf::ElementPtr _elem)
{
  // Populate the taskTab by parsing out SDF
  if (!_elem->HasElement("task_group"))
    return;

  int taskIndex = 0;
  int groupIndex = 0;

  // Create a button group. This group will hold all the task buttons.
  QButtonGroup *buttonGroup = new QButtonGroup();

  sdf::ElementPtr taskGroup = _elem->GetElement("task_group");

  // Process each task group, as specified in SDF
  while (taskGroup)
  {
    std::string taskGroupName = taskGroup->Get<std::string>("name");
    sdf::ElementPtr task = taskGroup->GetElement("task");

    // Create the button layout and frame
    QFrame *groupFrame = new QFrame();
    QGridLayout *groupLayout = new QGridLayout();
    groupFrame->setLayout(groupLayout);

    int count = 0;

    // Process each task in the group
    while (task)
    {
      // Read task information
      std::string id = task->Get<std::string>("id");
      std::string name = task->Get<std::string>("name");
      std::string instructions = task->Get<std::string>("instructions");
      std::string iconPath = common::SystemPaths::Instance()->FindFileURI(
          task->Get<std::string>("icon"));

      // Create a new button for the task
      TaskButton *taskButton = new TaskButton(name, id, taskIndex, groupIndex);
      taskButton->SetInstructions(instructions);

      // Listen to the task button press signal
      connect(taskButton, SIGNAL(SendTask(const int)),
         this, SLOT(OnTaskSent(const int)));

      int col = count % 4;
      int row = count / 4;

      // Add the button to the visual layout
      groupLayout->addWidget(taskButton, row, col);

      // Add the button to the button group (ensurce exclusive buttons)
      buttonGroup->addButton(taskButton);

      // Add an icon, if specified
      if (!iconPath.empty())
      {
        QPixmap iconPixmap(QString::fromStdString(iconPath));

        taskButton->setIcon(QIcon(iconPixmap));
        taskButton->setIconSize(QSize(60, 60));
        taskButton->setMinimumSize(80, 80);
        taskButton->setMaximumSize(100, 80);
      }

      // Set the first button checked, and set the instructions.
      if (this->taskList.empty())
      {
        instructionsView->setDocument(taskButton->Instructions());
        taskButton->setChecked(true);
      }

      this->taskList[taskIndex] = taskButton;

      task = task->GetNextElement();

      count++;
      taskIndex++;
    }

    this->taskTab->addTab(groupFrame, QString::fromStdString(taskGroupName));
    taskGroup = taskGroup->GetNextElement("task_group");
    groupIndex++;
  }


/*
  // Start/Stop button
  this->isTestRunning = false;
  this->startStopButton = new QPushButton();
  this->startStopButton->setText(QString("Start Test"));
  this->startStopButton->setStyleSheet(startButtonStyle);
  this->startStopButton->setMaximumWidth(this->handImgX*0.85);
  connect(this->startStopButton, SIGNAL(clicked()), this,
          SLOT(OnStartStopClicked()));

  // Clock
  this->digitalClock = new DigitalClock();
  connect(this->startStopButton, SIGNAL(clicked()), this->digitalClock,
          SLOT(OnStartStop()));
  this->digitalClock->setMaximumWidth(this->handImgX*0.5);

  QHBoxLayout *clockBox = new QHBoxLayout();
  clockBox->addStretch();
  clockBox->addWidget(digitalClock);
  clockBox->addStretch();
  QFrame *clockFrame = new QFrame();
  clockFrame->setLayout(clockBox);
  clockFrame->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(clockFrame);

  QHBoxLayout *startStopBox = new QHBoxLayout();
  startStopBox->addStretch();
  startStopBox->addWidget(startStopButton);
  startStopBox->addStretch();
  QFrame *startStopFrame = new QFrame();
  startStopFrame->setLayout(startStopBox);
  startStopFrame->setContentsMargins(0, 0, 0, 0);
  mainLayout->addWidget(startStopFrame);
  */
}

/////////////////////////////////////////////////
void HaptixGUIPlugin::OnTaskSent(const int _id)
{
  // Show the instructions to the user
  this->instructionsView->setDocument(this->taskList[_id]->Instructions());
  this->currentTaskId = _id;
  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnNextClicked()
{
  this->currentTaskId = (this->currentTaskId+1) % this->taskList.size();
  this->instructionsView->setDocument(
      this->taskList[this->currentTaskId]->Instructions());
  this->taskList[this->currentTaskId]->setChecked(true);
  this->taskTab->setCurrentIndex(this->taskList[this->currentTaskId]->Group());

  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());
}

////////////////////////////////////////////////
void HaptixGUIPlugin::PublishTaskMessage(const std::string &_taskId) const
{
  msgs::GzString msg;
  msg.set_data(_taskId);
  this->taskPub->Publish(msg);
}




/////////////////////////////////////////////////
// \todo: This function is inefficient. Think of ways to refactor--maybe a more
// efficient structure for storage/search.
bool HaptixGUIPlugin::OnKeyPress(common::KeyEvent /*_event*/)
{
  /*
  std::string text = _event.text;
  char key = text[0];
  // if key is in armCommands
  if(this->armCommands.find(key) != this->armCommands.end()){
    int index = this->armCommands[key].index;
    if(index >= 6){
      return false;
    }
    float inc = this->armCommands[key].increment;

    float pose_inc_args[6] = {0, 0, 0, 0, 0, 0};
    pose_inc_args[index] = inc;
    gazebo::math::Pose increment(gazebo::math::Vector3(pose_inc_args[0],
                                   pose_inc_args[1], pose_inc_args[2]),
                                 gazebo::math::Quaternion(pose_inc_args[3],
                                   pose_inc_args[4], pose_inc_args[5]));

    gazebo::math::Quaternion cameraRot =
                                gui::get_active_camera()->GetWorldRotation();
    if (index < 3)
    {
      increment.pos = cameraRot.RotateVector(increment.pos);
    }
    else
    {
      std::cout << "Increment.rot: " << increment.rot.GetAsEuler() << std::endl;
      std::cout << "camera rot: " << cameraRot.GetAsEuler() << std::endl;
      increment.rot = increment.rot*cameraRot;
      std::cout << "rotation after multiplication " << increment.rot.GetAsEuler() << std::endl;
    }

    gazebo::msgs::Pose msg;
    gazebo::msgs::Vector3d* vec_msg = msg.mutable_position();
    vec_msg->set_x(increment.pos.x);
    vec_msg->set_y(increment.pos.y);
    vec_msg->set_z(increment.pos.z);
    gazebo::msgs::Quaternion* quat_msg = msg.mutable_orientation();
    quat_msg->set_x(increment.rot.x);
    quat_msg->set_y(increment.rot.y);
    quat_msg->set_z(increment.rot.z);
    quat_msg->set_w(increment.rot.w);

    ignNode->Publish("/haptix/arm_pose_inc", msg);
    return true;
  }
  // if key is in handCommands
  if (this->handCommands.find(key) != this->handCommands.end())
  {
      int motor_index = this->handCommands[key].index;
      if (motor_index >= handDeviceInfo.nmotor)
      {
        return false;
      }

      float inc = this->handCommands[key].increment;
      handCommand.ref_pos[motor_index] += inc;

      //coupling_v1(&handCommand);
  }
  else if (this->graspCommands.find(key) != this->graspCommands.end())
  {
    std::string name = this->graspCommands[key];

    // Increment/decrement slider value corresponding to key
    grasps[name].SliderChanged(key, this->graspIncrement);
  }


  float sliderTotal = 0;
  // get total of slider values
  for (std::map<std::string, Grasp>::iterator it = grasps.begin();
       it != grasps.end(); it++)
  {
    if (it->second.sliderValue > 0)
    {
      sliderTotal += 1;
    }
  }

  hxCommand avgCommand;
  for (int j = 0; j < handDeviceInfo.nmotor; j++)
  {
    avgCommand.ref_pos[j] = 0;
  }

  for (std::map<std::string, Grasp>::iterator it = grasps.begin();
       it != grasps.end(); it++)
  {
    // Get the slider value and interpolate the grasp
    float sliderValue = it->second.sliderValue;

    std::vector<float> desiredGrasp = it->second.desiredGrasp;
    for (unsigned int j = 0; j < handDeviceInfo.nmotor; j++)
    {
      if(sliderTotal > 0 && j < desiredGrasp.size()){
        avgCommand.ref_pos[j] += sliderValue*desiredGrasp[j]/
                                                        (sliderTotal);
      }
    }
  }
  for (int j = 0; j < handDeviceInfo.nmotor; j++)
  {
    avgCommand.ref_pos[j] += this->handCommand.ref_pos[j];
  }

  // if (hx_update(hxGAZEBO, &avgCommand, &handSensor) != hxOK)
  // {
  //   printf("hx_update(): Request error.\n");
  // }
  */
  return true;
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnResetClicked()
{
  // Signal to the ArrangePlugin to set up the current task
  // this->digitalClock->StopClock();
  this->PublishTaskMessage(this->taskList[this->currentTaskId]->Id());
}

////////////////////////////////////////////////
void HaptixGUIPlugin::OnStartStopClicked()
{
  /*
  if(isTestRunning)
  {
    startStopButton->setStyleSheet(startButtonStyle);
    startStopButton->setText(QString("Start Test"));
    // Stop timer
  }
  else
  {
    startStopButton->setStyleSheet(stopButtonStyle);
    startStopButton->setText(QString("End Test"));
    // Start timer
  }
  isTestRunning = !isTestRunning;
  */
}


/*
void Grasp::SliderChanged(char key, float inc){
  if (key != this->incKey && key != this->decKey)
  {
    return;
  }
  int sign = key == this->incKey ? 1 : -1;
  this->sliderValue += sign*inc;
  this->sliderValue = this->sliderValue < 0 ? 0 : this->sliderValue;
  this->sliderValue = this->sliderValue > 1 ? 1 : this->sliderValue;
}
*/

/*
DigitalClock::DigitalClock(QWidget *_parent) : QLCDNumber(_parent)
{
  setSegmentStyle(Filled);
  setStyleSheet("font: 30px;");
  setDigitCount(8);

  lastStartTime = QTime(0, 0);
  //lastStartTime.start();

  running = false;
  QTimer *clock = new QTimer(this);
  connect(clock, SIGNAL(timeout()), this, SLOT(ShowTime()));
  clock->start(100);

  ShowTime();

  //resize(300, 60);
}

void DigitalClock::ShowTime()
{
  QString text("00:00:00");
  if(running)
  {
    QTime elapsedTime = QTime(0, 0, 0).addMSecs(lastStartTime.elapsed());
    text = elapsedTime.toString("hh:mm:ss");
  }

  display(text);
  update();
}


void DigitalClock::StopClock()
{
  this->running = false;
}

void DigitalClock::OnStartStop()
{
  running = !running;

  if (running)
  {
    // If we are now running, restart the time
    lastStartTime.restart();
    ShowTime();
  }
}
*/


//const hxAPLMotors mcp_indices[5] = {motor_little_mcp, motor_ring_mcp, motor_middle_mcp, motor_index_mcp, motor_thumb_mcp};

/*void coupling_v1(_hxCommand* cmd){
  //Version 1 Joint Coupling modeling

  //this relies on the ordering of enums, bit messy
  for(int k = 0; k < 5; k++){
    //Check if slider was changeded

    hxAPLMotors mcp = mcp_indices[k];

    float mcp_commanded = cmd->ref_pos[mcp];
    if(mcp_commanded <= 0){
      if(mcp == motor_thumb_mcp){
        cmd->ref_pos[mcp+1] = 0; //thumb dip
      } else {
        cmd->ref_pos[mcp+1] = 0; //pip
        cmd->ref_pos[mcp-1] = 0; //dip
      }
    } else {
      if(mcp == motor_thumb_mcp){
        cmd->ref_pos[mcp+1] = 8/9.0*mcp_commanded; //thumb dip
      } else {
        cmd->ref_pos[mcp+1] = 10/9.0*mcp_commanded; //pip
        cmd->ref_pos[mcp-1] = 8/9.0*mcp_commanded; //dip
      }
    }
  }
}*/

