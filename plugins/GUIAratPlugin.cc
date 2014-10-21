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

void GUIAratPlugin::InitializeHandView(QLayout* mainLayout)
{
  // Create a QGraphicsView to draw the finger force contacts
  this->handScene = new QGraphicsScene(QRectF(0, 0, this->handImgX,
                                                    this->handImgY));
  QGraphicsView *handView = new QGraphicsView(this->handScene);

  // Load the hand image
  QPixmap* handImg = new QPixmap(QString(this->handImgFilename.c_str()));
  QGraphicsPixmapItem* handItem = new QGraphicsPixmapItem(*handImg);

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
  handView->show();

  // Add the frame to the main layout
  handView->setMaximumSize(this->handImgX+10, this->handImgY+10);
  mainLayout->addWidget(handView);
}

void GUIAratPlugin::InitializeTaskView(QLayout* mainLayout,
                                       sdf::ElementPtr elem,
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
      QTextDocument* instructionsDocument = new QTextDocument(QString(instructions.c_str()));
      taskButton->SetTaskInstructionsDocument(instructionsDocument);
      taskButton->SetIndex(taskList.size());

      connect(taskButton, SIGNAL(SendTask(std::string, QTextDocument*, int)), this, SLOT(OnTaskSent(const std::string&, QTextDocument*, const int)));

      int col = i%3;
      int row = i/3;
      buttonLayout->addWidget(taskButton, row, col);

      if(icon_path.compare("none") != 0){
        QPixmap icon_picture(QString(paths->FindFileURI(icon_path).c_str()));
        
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
  QToolButton *resetButton = new QToolButton();
  resetButton->setText(QString("Reset Test"));
  resetButton->setStyleSheet("border:1px solid #ffffff");
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
  cycleButtonLayout->addWidget(resetButton);
  resetButton->setMaximumWidth(this->handImgX/2);

  QToolButton *nextButton = new QToolButton();
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
  
  //std::cout << "getting sdf" << std::endl;
  // Parameters for sensor contact visualization
  sdf::SDF parameters;
  parameters.SetFromString(sdfString);
  sdf::ElementPtr elem;
  //assert(parameters.root->HasElement("world");
  elem = parameters.root->GetElement("world");
  //assert(elem->HasElement("plugin");
  elem = elem->GetElement("plugin");
  //assert(elem->HasElement("circleSize");
  elem->GetElement("circleSize")->GetValue()->Get(this->circleSize);
  //assert(elem->HasElement("forceMin");
  elem->GetElement("forceMin")->GetValue()->Get(this->forceMin);
  elem->GetElement("forceMax")->GetValue()->Get(this->forceMax);
  elem->GetElement("colorMin")->GetValue()->Get(this->colorMin);
  elem->GetElement("colorMax")->GetValue()->Get(this->colorMax);
  elem->GetElement("handSide")->GetValue()->Get(this->handSide);

  math::Vector2d handImgDims;
  elem->GetElement("handImgDimensions")->GetValue()->Get(handImgDims);
  this->handImgX = handImgDims[0];
  this->handImgY = handImgDims[1];

  elem->GetElement("iconDimensions")->GetValue()->Get(this->iconSize);

  //std::cout << "getting contact names" << std::endl;
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
        contact = contact->GetNextElement();
      }
    }
  }

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->InitializeHandView(mainLayout);

  this->InitializeTaskView(mainLayout, elem, paths);

  // Remove margins to reduce space
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->setMaximumWidth(this->handImgX+10);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->taskPub = this->node->Advertise<msgs::GzString>("~/control");

  this->ignNode = new ignition::transport::Node("haptix");
  ignNode->Advertise("arm_pose_inc");

  // Parse some SDF to get the arm teleop commands
  //std::cout << "getting commands" << std::endl;
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
      //std::cout << "got hand button: " << button << std::endl;
    } else if(name.substr(0, 3).compare("arm") == 0){
      //std::cout << "got arm button: " << button << std::endl;
      this->armCommands[button] = KeyCommand(button, name, increment);
    }
    buttonNames[name].push_back(button);

    command = command->GetNextElement();
  }

  //std::cout << "Getting indices" << std::endl;
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
      //std::cout << "Got index: " << num << " for button " << button << std::endl;
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

  this->connections.push_back(event::Events::ConnectPreRender(boost::bind(&GUIAratPlugin::PreRender, this)));

  gui::get_active_camera()->SetViewController(
                                rendering::FPSViewController::GetTypeString());

  gui::KeyEventHandler::Instance()->SetAutoRepeat(true);
  gui::KeyEventHandler::Instance()->AddPressFilter("arat_gui",
                          boost::bind(&GUIAratPlugin::OnKeyPress, this, _1));
}


/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
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
    std::string rawString = msg->contact(0).collision2();
    size_t end_idx = rawString.find("::", 6);
    
    std::string fingerName = rawString.substr(6, end_idx-6);
    if(this->contactPoints.find(fingerName) != this->contactPoints.end())
    {
      //std::cout << "Pushing to msg queue" << std::endl;
      this->msgQueue.push(ContactsWrapper(msg, fingerName));
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
      
      std::cout << "drawing circle color: " << colorArray[0] << ", " << colorArray[1] << ", " << colorArray[2] << std::endl;
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

/*void GUIAratPlugin::keyPressEvent(QKeyEvent *_event)
{
  std::cout << "got key " << _event->key() << std::endl;
  std::string text = _event->text().toStdString();
  this->OnKeyEvent(text[0], false);
}

void GUIAratPlugin::keyReleaseEvent(QKeyEvent *_event)
{
  std::string text = _event->text().toStdString();
  this->OnKeyEvent(text[0], true);
}*/

bool GUIAratPlugin::OnKeyPress(common::KeyEvent _event)
{
  std::string text = _event.text;
  //std::cout << "got key " << text <<  std::endl;
  char key = text[0];
  // if key is in armCommands
  if(this->armCommands.find(key) != this->armCommands.end()){
    int index = this->armCommands[key].index;
    if(index >= 6){
      return false;
    }
    float inc = this->armCommands[key].increment;
    float pose_inc_args[6] = {0, 0, 0, 0, 0, 0};
    //if (!release){
    pose_inc_args[index] = inc;
    //}
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
  }
  // if key is in handCommands
  else if (this->handCommands.find(key) != this->handCommands.end())
  {
      int motor_index = this->handCommands[key].index;
      if(motor_index >= handDeviceInfo.nmotor){
        return false;
      }
      /*if(release){
        for(int i = 0; i < handDeviceInfo.nmotor; i++){
          handCommand.ref_vel[i] = 0;
        }
      } else {

        handCommand.ref_vel[motor_index] += inc*100;
      }*/

      float inc = this->handCommands[key].increment;
      handCommand.ref_pos[motor_index] += inc;
      //handCommand.ref_vel[motor_index] += inc;
      /*for(int i = 0; i < handDeviceInfo.nmotor; i++){
        handCommand.ref_pos[i] = handSensor.joint_pos[i];
        std::cout << "hand sensor " << i << ": " << handSensor.joint_pos[i] << std::endl;
      }*/

      coupling_v1(&handCommand);

      if (hx_update(hxGAZEBO, &handCommand, &handSensor) != hxOK){
        printf("hx_update(): Request error.\n");
      }
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
  PublishTaskMessage(this->taskList[currentTaskIndex]);
}

void GUIAratPlugin::OnNextClicked()
{
  this->currentTaskIndex = (this->currentTaskIndex+1) % this->taskList.size();
  PublishTaskMessage(this->taskList[this->currentTaskIndex]);
  this->instructionsView->setDocument(
                               this->instructionsList[this->currentTaskIndex]);
}

void GUIAratPlugin::OnTaskSent(const std::string& id,
                               QTextDocument* instructions, const int index)
{
  // Show the instructions to the user
  this->instructionsView->setDocument(instructions);
  PublishTaskMessage(id);
  this->currentTaskIndex = index;
}
