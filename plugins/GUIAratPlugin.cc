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
#include "teleop.h"

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
   
  sdf::ElementPtr taskGroup = elem->GetElement("taskGroup");
  while(taskGroup){
    std::string taskGroupName;
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

  this->ignNode = ignition::transport::NodePtr(
                                new ignition::transport::Node("haptix"));
  ignNode->Advertise("arm_pose_inc");

  // Parse some SDF to get the arm teleop commands
  ElementPtr command = elem->GetElement("commands");
  command = commands->GetElement("command");
  while(command){
    std::string button;
    std::string name;
    float increment;
    command->GetAttribute("button")->Get(button);
    command->GetAttribute("name")->Get(name);
    command->GetAttribute("increment")->Get(increment);

    if(name.substr(0, "motor".size()).compare("motor") == 0){
      handMappings[button] = KeyCommand(button, name, increment);
    } else if(name.substr(0, "arm".size()).compare("arm") == 0){
      armMappings[button] = KeyCommand(button, name, increment);
    }

    command = command->GetNextElement();
  }

  ElementPtr index = elem->GetElement("indices");
  index = index->GetElement("index");
  while(index){
    std::string name;
    int num;
    index->GetAttribute("name")->Get(name);
    index->GetAttribute("num")->Get(num);
    if(handMappings.find(name) != handMappings.end()){
      handMappings[name].index = num;
    } else if (armMappings.find(name) != handMappings.end()){
      armMappings[name].index = num;
    }
    index = index->GetNextElement();
  }

  // Set up dat subscriber
  KeyEventHandler::Instance()->AddReleaseFilter("Commands",
                        boost::bind(&GUIAratPlugin::OnKeyRelease,  this, _1));

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
}

/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
}

/*void GUIAratPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  _visual->GetScene()->GetUserCamera(0)->SetViewController(
                                rendering::FPSViewController::GetTypeString());
}

void GUIAratPlugin::Init()
{

}*/

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
    int numContacts = msg->contact_size();
    if(numContacts > 0){
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

void GUIAratPlugin::OnKeyRelease(const common::KeyEvent &_event)
{
  // convert key to a string
  
  
  // if key is in armCommands
  if(this->armCommands.find(key) != armCommands.end()){
    int index = armMappings[key].index;

    float pose_inc_args[6] = {0, 0, 0, 0, 0, 0};
    pose_inc_args[index] = inc;
    math::Pose pose_inc(pose_inc_args[0], pose_inc_args[1],
                                          pose_inc_args[2], pose_inc_args[3],
                                          pose_inc_args[4], pose_inc_args[5]);
    gazebo::msgs::Pose msg;
    gazebo::msgs::Vector3d* vec_msg;
    vec_msg = msg.mutable_position();
    vec_msg->set_x(pose_inc.Pos().X());
    vec_msg->set_y(pose_inc.Pos().Y());
    vec_msg->set_z(pose_inc.Pos().Z());
    gazebo::msgs::Quaternion* quat_msg;
    quat_msg = msg.mutable_orientation();
    quat_msg->set_x(pose_inc.Rot().X());
    quat_msg->set_y(pose_inc.Rot().Y());
    quat_msg->set_z(pose_inc.Rot().Z());
    quat_msg->set_w(pose_inc.Rot().W());

    ignNode.Publish("/haptix/arm_pose_inc", msg);

  } else if (this->handCommands.find(key) != handCommands.end()){
    // if key is in handCommands
      unsigned int motor_index = handMappings[key].index;
      hxCommand cmd;
      cmd.ref_pos[motor_index] += inc;

      coupling_v1(&cmd);

      if (hx_update(hxGAZEBO, &cmd, &sensor) != hxOK)
        printf("hx_update(): Request error.\n");
      else
        cmd.timestamp = sensor.timestamp;
  }


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
