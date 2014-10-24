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
#include <gazebo/rendering/FPSViewController.hh>
#include "GUIAratPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIAratPlugin)

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
 
QTaskButton::QTaskButton()
{
  connect(this, SIGNAL(clicked()), this, SLOT(OnButton()));
}

void QTaskButton::SetTaskId(const std::string& task_id)
{
  this->id = task_id;
}

void QTaskButton::SetTaskInstructionsDocument(QTextDocument* instr)
{
  this->instructions = instr;
}

void QTaskButton::SetIndex(const int i)
{
  this->index = i;
}

void QTaskButton::OnButton()
{
  emit SendTask(this->id, this->instructions, this->index);
}

DigitalClock::DigitalClock(QWidget *parent) : QLCDNumber(parent)
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

void GUIAratPlugin::InitializeHandView()
{
  // Create a QGraphicsView to draw the finger force contacts
  this->handScene = new QGraphicsScene(QRectF(0, 0, this->handImgX,
                                                    this->handImgY));
  QGraphicsView *handView = new QGraphicsView(this->handScene);

  // Load the hand image
  QPixmap handImg = QPixmap(QString(this->handImgFilename.c_str()));
  handImg = handImg.scaled(this->handImgX, this->handImgY);
  QGraphicsPixmapItem* handItem = new QGraphicsPixmapItem(handImg);

  // Draw the hand on the canvas
  this->handScene->addItem(handItem);

  // Preallocate QGraphicsItems for each contact point
  for(std::map<std::string, math::Vector2d>::iterator it =
        this->contactPoints.begin(); it != this->contactPoints.end(); it++)
  {
    std::string fingerName = it->first;
    int xpos = this->contactPoints[fingerName][0];
    int ypos = this->contactPoints[fingerName][1];
    this->contactGraphicsItems[fingerName] =
                  new QGraphicsEllipseItem(xpos, ypos,
                                           this->circleSize, this->circleSize);
    this->handScene->addItem(this->contactGraphicsItems[fingerName]);

    this->contactGraphicsItems[fingerName]
                                 ->setBrush(QBrush(QColor(255, 255, 255, 0)));
    this->contactGraphicsItems[fingerName]
                                 ->setPen(QPen(QColor(153, 153, 153, 255)));
  }

  this->handScene->update();
  handView->setMinimumSize(this->handImgX, this->handImgY);
  handView->setMaximumSize(this->handImgX+5, this->handImgY+5);

  mainLayout->addWidget(handView);
  handView->show();
}

void GUIAratPlugin::InitializeTaskView(sdf::ElementPtr elem,
                                       common::SystemPaths* paths)
{
  QVBoxLayout *taskLayout = new QVBoxLayout();
  taskLayout->setContentsMargins(0, 0, 0, 0);
  
  QTabWidget *tabWidget = new QTabWidget();
  
  // Populate the tabWidget by parsing out SDF
  this->instructionsView = new QTextEdit();
  this->instructionsView->setReadOnly(true);
  this->instructionsView->setMaximumHeight(handImgY/3);
   
  if(! elem->HasElement("taskGroup") ){
    return;
  }
  sdf::ElementPtr taskGroup = elem->GetElement("taskGroup");
  //std::cout << "Getting tasks" << std::endl;
  while(taskGroup){
    std::string taskGroupName;
    if(!taskGroup->HasAttribute("name")){
      break;
    }
    taskGroup->GetAttribute("name")->Get(taskGroupName);

    QGroupBox *buttonGroup = new QGroupBox();
    QGridLayout *buttonLayout = new QGridLayout();
    
    int i = 0;
    sdf::ElementPtr task = taskGroup->GetElement("task");
    while(task){
      std::string id;
      std::string name;
      std::string icon_path;
      std::string instructions;
      task->GetAttribute("id")->Get(id);
      task->GetAttribute("name")->Get(name);
      task->GetAttribute("icon")->Get(icon_path);
      task->GetAttribute("instructions")->Get(instructions);

      QTaskButton *taskButton = new QTaskButton();
      taskButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      taskButton->setMaximumWidth(this->handImgX/3);
      taskButton->setMaximumHeight(this->handImgX/3);
      taskButton->resize(this->handImgX/3, this->handImgY/3);
      taskButton->setText(QString(name.c_str()));
      taskButton->SetTaskId(id);
      QTextDocument* instructionsDocument =
                            new QTextDocument(QString(instructions.c_str()));
      taskButton->SetTaskInstructionsDocument(instructionsDocument);
      taskButton->SetIndex(taskList.size());

      connect(taskButton, SIGNAL(SendTask(std::string, QTextDocument*, int)),
         this, SLOT(OnTaskSent(const std::string&, QTextDocument*, const int)));

      int col = i%3;
      int row = i/3;
      buttonLayout->addWidget(taskButton, row, col);

      if(icon_path.compare("none") != 0){
        QPixmap icon_picture(QString(paths->FindFileURI(icon_path).c_str()));
        icon_picture = icon_picture.scaled(iconSize[0], iconSize[1]);
        
        taskButton->setIcon(QIcon(icon_picture));
        taskButton->setIconSize(QSize(iconSize[0], iconSize[1]));
        taskButton->setMinimumSize(iconSize[0]+20, iconSize[1]+30);
      }
      if(taskList.empty()){
        instructionsView->setDocument(instructionsDocument);
      }
      taskList.push_back(id);
      this->instructionsList.push_back(instructionsDocument);

      task = task->GetNextElement();
      i++;
    }
    buttonGroup->setMinimumWidth(handImgX);
    buttonGroup->setContentsMargins(0, 0, 0, 0);
    buttonGroup->setLayout(buttonLayout);
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->setSpacing(0);

    tabWidget->addTab((QWidget*) buttonGroup, QString(taskGroupName.c_str()));

    taskGroup = taskGroup->GetNextElement();
  }

  this->currentTaskIndex = 0;

  QFrame *taskFrame = new QFrame();
  tabWidget->setContentsMargins(0, 0, 0, 0);
  taskLayout->addWidget(tabWidget);
  taskLayout->addWidget(instructionsView);

  QHBoxLayout* cycleButtonLayout = new QHBoxLayout();
  QPushButton *resetButton = new QPushButton();
  resetButton->setText(QString("Reset Test"));
  resetButton->setStyleSheet("border:1px solid #ffffff");
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
  cycleButtonLayout->addWidget(resetButton);
  resetButton->setMaximumWidth(this->handImgX/2);

  QPushButton *nextButton = new QPushButton();
  nextButton->setText(QString("Next Test"));
  nextButton->setStyleSheet("border:1px solid #ffffff");
  connect(nextButton, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
  cycleButtonLayout->addWidget(nextButton);
  nextButton->setMaximumWidth(this->handImgX/2);

  QFrame* cycleButtonFrame = new QFrame;
  cycleButtonFrame->setLayout(cycleButtonLayout);
  taskLayout->addWidget(cycleButtonFrame);

  taskFrame->setLayout(taskLayout);
  mainLayout->addWidget(taskFrame);

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
}

/////////////////////////////////////////////////
GUIAratPlugin::GUIAratPlugin()
  : GUIPlugin()
{

  // Read parameters
  common::SystemPaths* paths = common::SystemPaths::Instance();
  this->handImgFilename = paths->FindFileURI
                                  ("file://media/gui/arat/handsim.png");
  this->configFilename = paths->FindFileURI
                                  ("file://media/gui/arat/GUIAratPlugin.sdf");

  
  std::ifstream fileinput(this->configFilename.c_str());
  std::stringstream inputStream;
  inputStream << fileinput.rdbuf();
  std::string sdfString = inputStream.str();
  fileinput.close();
  
  // Parameters for sensor contact visualization
  sdf::SDF parameters;
  parameters.SetFromString(sdfString);
  sdf::ElementPtr elem;
  elem = parameters.root->GetElement("world");
  elem = elem->GetElement("plugin");
  elem->GetElement("circleSize")->GetValue()->Get(this->circleSize);

  elem->GetElement("forceMin")->GetValue()->Get(this->forceMin);
  elem->GetElement("forceMax")->GetValue()->Get(this->forceMax);
  elem->GetElement("colorMin")->GetValue()->Get(this->colorMin);
  elem->GetElement("colorMax")->GetValue()->Get(this->colorMax);
  elem->GetElement("handSide")->GetValue()->Get(this->handSide);
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

  // Get contact names
  if(elem->HasElement("contacts")){
    sdf::ElementPtr contact = elem->GetElement("contacts");
    contact = contact->GetElement("contact");
    while(contact){
      if(contact->HasAttribute("name")){
        std::string contactName;
        contact->GetAttribute("name")->Get(contactName);
        // Get the position of the contact
        contact->GetValue()->Get(this->contactPoints[contactName]);
        this->contactPoints[contactName]*=this->GUIScaleFactor;
        contact = contact->GetNextElement();
      }
    }
  }

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");
  mainLayout = new QVBoxLayout();
  this->setPalette(QPalette(QColor(255, 255, 255, 0)));
  
  this->InitializeHandView();

  this->InitializeTaskView(elem, paths);


  // Remove margins to reduce space
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(0);
  this->setLayout(mainLayout);

  this->setMaximumWidth(this->handImgX+10);
  this->setMinimumHeight(this->handImgY*2.6);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->taskPub = this->node->Advertise<msgs::GzString>("~/arrange");

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

  // Get predefined grasps
  sdf::ElementPtr grasp = elem->GetElement("grasps");
  grasp->GetElement("increment")->GetValue()->Get(this->graspIncrement);
  grasp = grasp->GetElement("grasp");
  while(grasp){
    std::string name;
    char inc_key;
    char dec_key;
    grasp->GetAttribute("name")->Get(name);
    grasp->GetAttribute("inc_key")->Get(inc_key);
    grasp->GetAttribute("dec_key")->Get(dec_key);
    this->grasps[name] = Grasp(inc_key, dec_key);
    std::string graspBuffer;
    grasp->GetValue()->Get(graspBuffer);
    std::istringstream iss(graspBuffer);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    for(unsigned int i = 0; i < tokens.size(); i++)
    {
      grasps[name].desiredGrasp.push_back(stof(tokens[i]));
    }
    this->graspCommands[inc_key] = name;
    this->graspCommands[dec_key] = name;
    grasp = grasp->GetNextElement();
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
  if (hx_getdeviceinfo(hxGAZEBO, &handDeviceInfo) != hxOK)
  {
    std::cout << "hx_getdeviceinfo(): Request error. Cannot control hand."
              << std::endl;
  }
  for(int i = 0; i < handDeviceInfo.nmotor; i++)
  {
    handCommand.ref_pos[i] = 0;
  }
  if(hx_update(hxGAZEBO, &handCommand, &handSensor) != hxOK ){
    std::cout << "hx_update(): Request error.\n" << std::endl;
  }

  // Set up an array of subscribers for each contact sensor
  for(std::map<std::string, math::Vector2d>::iterator it =
        this->contactPoints.begin(); it != this->contactPoints.end(); it++)
  {
    std::string topic = this->getTopicName(it->first);
    transport::SubscriberPtr sub = this->node->Subscribe(topic,
                                   &GUIAratPlugin::OnFingerContact, this);
    this->contactSubscribers.push_back(sub);
  }

  this->connections.push_back(event::Events::ConnectPreRender(
                              boost::bind(&GUIAratPlugin::PreRender, this)));

  gui::KeyEventHandler::Instance()->SetAutoRepeat(true);
  gui::KeyEventHandler::Instance()->AddPressFilter("arat_gui",
                          boost::bind(&GUIAratPlugin::OnKeyPress, this, _1));
}


/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
}

std::string parseString(const std::string& rawString){
  size_t end_idx = rawString.find("::", 6);
  return rawString.substr(6, end_idx-6);
}

std::string GUIAratPlugin::getTopicName(const std::string& fingerName)
{
  std::string topicName = this->handSide+fingerName;
  return "~/mpl/"+topicName+"/"+topicName+"_contact_sensor";
}

void GUIAratPlugin::OnFingerContact(ConstContactsPtr &msg){
  // Parse out the finger name
  // Format is: mpl::r<finger name>::r<finger name>_collision
  // start at index 6
  if (msg->contact_size() > 0)
  {
    std::string collisionName1 = parseString(msg->contact(0).collision1());
    std::string collisionName2 = parseString(msg->contact(0).collision2());
    if (this->contactPoints.find(collisionName1) != this->contactPoints.end())
    {
      this->msgQueue.push(ContactsWrapper(msg, collisionName1));
    }
    else if (this->contactPoints.find(collisionName1) !=
             this->contactPoints.end())
    {
      this->msgQueue.push(ContactsWrapper(msg, collisionName2));
    }
  }
}

void GUIAratPlugin::PreRender()
{
  // Draw the contacts as empty gray circles
  for(std::map<std::string, math::Vector2d>::iterator it =
        this->contactPoints.begin(); it != this->contactPoints.end(); it++)
  {
    std::string fingerName = it->first;

    this->contactGraphicsItems[fingerName]->
                                    setBrush(QBrush(QColor(255, 255, 255, 1)));
    this->contactGraphicsItems[fingerName]->
                                    setPen(QPen(QColor(153, 153, 153, 255)));
  }

  //Clear queued messages and draw them
  while(!this->msgQueue.empty())
  {
    ContactsWrapper wrapper = this->msgQueue.front();
    ConstContactsPtr msg = wrapper.msg;
    std::string fingerName = wrapper.name;
    this->msgQueue.pop();
    if(msg->contact_size() > 0){
      // Calculate contact force
      msgs::Vector3d forceVector = msg->contact(0).wrench(0).
                                                  body_1_wrench().force();
      float f = math::Vector3(forceVector.x(), forceVector.y(),
                                              forceVector.z()).GetLength();
      float forceRange = this->forceMax - this->forceMin;

      float colorArray[3];
      for(int i = 0; i < 3; i++)
      {
        float colorRange = this->colorMax[i] - this->colorMin[i];
        colorArray[i] = colorRange/forceRange*f + this->colorMin[i];
        if(colorArray[i] > this->colorMin[i])
        {
          colorArray[i] = this->colorMin[i];
        }
        else if (colorArray[i] < this->colorMax[i])

          colorArray[i] = this->colorMax[i];
        }
      
      QBrush color(QColor(colorArray[0], colorArray[1], colorArray[2]));
      
      this->contactGraphicsItems[fingerName]->setBrush(color);
      this->contactGraphicsItems[fingerName]->setPen(QPen(QColor(0, 0, 0, 0)));
    }
  }
}


const hxAPLMotors mcp_indices[5] = {motor_little_mcp, motor_ring_mcp, motor_middle_mcp, motor_index_mcp, motor_thumb_mcp};

void coupling_v1(_hxCommand* cmd){
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
}

// TODO: This function is inefficient. Think of ways to refactor--maybe a more
// efficient structure for storage/search.
bool GUIAratPlugin::OnKeyPress(common::KeyEvent _event)
{
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
    gazebo::math::Quaternion quat(pose_inc_args[3],
                                 pose_inc_args[4], pose_inc_args[5]);
    gazebo::msgs::Pose msg;
    gazebo::msgs::Vector3d* vec_msg = msg.mutable_position();
    vec_msg->set_x(pose_inc_args[0]);
    vec_msg->set_y(pose_inc_args[1]);
    vec_msg->set_z(pose_inc_args[2]);
    gazebo::msgs::Quaternion* quat_msg = msg.mutable_orientation();
    quat_msg->set_x(quat.x);
    quat_msg->set_y(quat.y);
    quat_msg->set_z(quat.z);
    quat_msg->set_w(quat.w);

    ignNode->Publish("/haptix/arm_pose_inc", msg);
    return true;
  }
  // if key is in handCommands
  if (this->handCommands.find(key) != this->handCommands.end())
  {
      int motor_index = this->handCommands[key].index;
      if(motor_index >= handDeviceInfo.nmotor)
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
 
  if (hx_update(hxGAZEBO, &avgCommand, &handSensor) != hxOK)
  {
    printf("hx_update(): Request error.\n");
  }
  return true;
}

////////////////////////////////////////////////

void GUIAratPlugin::PublishTaskMessage(const std::string& task_name)
{
  msgs::GzString msg;
  msg.set_data(task_name);
  this->taskPub->Publish(msg);

}

void GUIAratPlugin::OnResetClicked()
{
  // Signal to the ArrangePlugin to set up the current task
  this->digitalClock->StopClock();
  PublishTaskMessage(this->taskList[currentTaskIndex]);
}

void GUIAratPlugin::OnNextClicked()
{
  this->currentTaskIndex = (this->currentTaskIndex+1) % this->taskList.size();
  PublishTaskMessage(this->taskList[this->currentTaskIndex]);
  this->instructionsView->setDocument(
                               this->instructionsList[this->currentTaskIndex]);
}


void GUIAratPlugin::OnStartStopClicked(){
  if(isTestRunning){
    startStopButton->setStyleSheet(startButtonStyle);
    startStopButton->setText(QString("Start Test"));
    // Stop timer
  } else {
    startStopButton->setStyleSheet(stopButtonStyle);
    startStopButton->setText(QString("End Test"));
    // Start timer
  }
  isTestRunning = !isTestRunning;
}

void GUIAratPlugin::OnTaskSent(const std::string& id,
                               QTextDocument* instructions, const int index)
{
  // Show the instructions to the user
  this->instructionsView->setDocument(instructions);
  PublishTaskMessage(id);
  this->currentTaskIndex = index;
}
