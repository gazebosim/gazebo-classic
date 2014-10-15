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
#include "GUIAratPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIAratPlugin)

QTaskButton::QTaskButton(){
  connect(this, SIGNAL(clicked()), this, SLOT(OnButton()));
}

void QTaskButton::SetTaskId(std::string task_id){
  this->id = task_id;
}

void QTaskButton::SetTaskInstructionsDocument(QTextDocument* instr){
  this->instructions = instr;
}

void QTaskButton::SetIndex(int i){
  this->index = i;
}

void QTaskButton::OnButton(){
  emit SendTask(this->id, this->instructions, this->index);
}

void GUIAratPlugin::InitializeHandView(QLayout* mainLayout){

  // Create a QGraphicsView to draw the finger force contacts
  this->handScene = new QGraphicsScene(QRectF(0, 0, handImgX, handImgY));
  QGraphicsView *handView = new QGraphicsView(handScene);

  // Load the hand image
  QPixmap* handImg = new QPixmap(QString(handImgFilename.c_str()));
  QGraphicsPixmapItem* handItem = new QGraphicsPixmapItem(*handImg);

  // Draw the hand on the canvas
  this->handScene->addItem(handItem);

  // Preallocate QGraphicsItems for each contact point
  for(std::map<std::string, math::Vector2d>::iterator it =
        finger_points.begin(); it != finger_points.end(); it++){
    std::string fingerName = it->first;
    int xpos = finger_points[fingerName][0];
    int ypos = finger_points[fingerName][1];
    this->contactGraphicsItems[fingerName] =
                  new QGraphicsEllipseItem(xpos, ypos, circleSize, circleSize);
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
                                       common::SystemPaths* paths){
  QVBoxLayout *taskLayout = new QVBoxLayout();
  taskLayout->setContentsMargins(0, 0, 0, 0);
  
  QTabWidget *tabWidget = new QTabWidget();
  
  // Populate the tabWidget by parsing out SDF
  instructionsView = new QTextEdit();
  instructionsView->setReadOnly(true);
  instructionsView->setMaximumHeight(handImgY/3);
   
  sdf::ElementPtr taskGroup = elem->GetElement("taskGroup");
  while(taskGroup){
    std::string taskGroupName;
    taskGroup->GetAttribute("name")->Get(taskGroupName);
    // Create a QWidget for the task group
    // Insert this widget into the tabWidget
    QGroupBox *buttonGroup = new QGroupBox();
    QGridLayout *buttonLayout = new QGridLayout();
    
    sdf::ElementPtr task = taskGroup->GetElement("task");
    int i = 0;

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
      taskButton->setMaximumWidth(handImgX/3);
      taskButton->setMaximumHeight(handImgX/3);
      taskButton->resize(handImgX/3, handImgY/3);
      taskButton->setText(QString(name.c_str()));
      taskButton->SetTaskId(id);
      QTextDocument* instructionsDocument = new QTextDocument(QString(instructions.c_str()));
      taskButton->SetTaskInstructionsDocument(instructionsDocument);
      taskButton->SetIndex(taskList.size());

      connect(taskButton, SIGNAL(SendTask(std::string, QTextDocument*, int)), this, SLOT(OnTaskSent(std::string, QTextDocument*, int)));

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

  currentTaskIndex = 0;

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
  resetButton->setMaximumWidth(handImgX/2);
  QToolButton *nextButton = new QToolButton();
  nextButton->setText(QString("Next Test"));
  nextButton->setStyleSheet("border:1px solid #ffffff");
  connect(nextButton, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
  cycleButtonLayout->addWidget(nextButton);
  nextButton->setMaximumWidth(handImgX/2);
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
  this->handImgFilename = paths->FindFileURI("file://media/gui/arat/handsim.png");
  this->configFilename = paths->FindFileURI("file://media/gui/arat/GUIAratPlugin.sdf");

  sdf::SDF parameters;
  
  std::ifstream fileinput(this->configFilename.c_str());
  std::stringstream inputStream;
  inputStream << fileinput.rdbuf();
  std::string sdfString = inputStream.str();
  fileinput.close();
  
  // Parameters for sensor contact visualization
  parameters.SetFromString(sdfString);
  sdf::ElementPtr elem;
  //assert(parameters.root->HasElement("world");
  elem = parameters.root->GetElement("world");
  //assert(elem->HasElement("plugin");
  elem = elem->GetElement("plugin");
  //assert(elem->HasElement("circleSize");
  elem->GetElement("circleSize")->GetValue()->Get(circleSize);
  //assert(elem->HasElement("forceMin");
  elem->GetElement("forceMin")->GetValue()->Get(forceMin);
  elem->GetElement("forceMax")->GetValue()->Get(forceMax);
  elem->GetElement("colorMin")->GetValue()->Get(colorMin);
  elem->GetElement("colorMax")->GetValue()->Get(colorMax);
  elem->GetElement("handSide")->GetValue()->Get(handSide);

  math::Vector2d handImgDims;
  elem->GetElement("handImgDimensions")->GetValue()->Get(handImgDims);
  handImgX = handImgDims[0];
  handImgY = handImgDims[1];

  elem->GetElement("iconDimensions")->GetValue()->Get(iconSize);

  // Get contact names
  if(elem->HasElement("contacts")){
    sdf::ElementPtr contact = elem->GetElement("contacts");
    contact = contact->GetElement("contact");
    while(contact){
      if(contact->HasAttribute("name")){
        std::string contactName;
        contact->GetAttribute("name")->Get(contactName);
        // Get the position of the contact
        contact->GetValue()->Get(finger_points[contactName]);
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
  this->taskNum = 0;
  this->maxTaskCount = 10;


  // Set up an array of subscribers for each contact sensor
  
  for(std::map<std::string, math::Vector2d>::iterator it =
        finger_points.begin(); it != finger_points.end(); it++){
    std::string topic = this->getTopicName(it->first);
    transport::SubscriberPtr sub = this->node->Subscribe(topic,
                                   &GUIAratPlugin::OnFingerContact, this);
    contactSubscribers.push_back(sub);
  }

  this->connections.push_back(event::Events::ConnectPreRender(boost::bind(&GUIAratPlugin::PreRender, this)));

}

/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
}

std::string GUIAratPlugin::getTopicName(std::string fingerName){
  std::string topicName = this->handSide+fingerName;
  return "~/mpl/"+topicName+"/"+topicName+"_contact_sensor";
}

void GUIAratPlugin::OnFingerContact(ConstContactsPtr &msg){
  // Parse out the finger name
  // Format is: mpl::r<finger name>::r<finger name>_collision
  // start at index 6
  if (msg->contact_size() > 0){
    std::string rawString = msg->contact(0).collision2();
    size_t end_idx = rawString.find("::", 6);
    
    std::string fingerName = rawString.substr(6, end_idx-6);
    std::cout << "parsed finger name: " << fingerName << std::endl;
    if(this->finger_points.find(fingerName) != this->finger_points.end()){
      this->msgQueue.push(ContactsWrapper(msg, fingerName));
    }
  }
}

void GUIAratPlugin::PreRender(){
  // Remove items from the scene
  for(std::map<std::string, math::Vector2d>::iterator it =
        finger_points.begin(); it != finger_points.end(); it++){

    std::string fingerName = it->first;

    this->contactGraphicsItems[fingerName]->setBrush(QBrush(QColor(255, 255, 255, 1)));
    this->contactGraphicsItems[fingerName]->setPen(QPen(QColor(153, 153, 153, 255)));

  }

  //Clear queued messages and draw them
  while( !this->msgQueue.empty()){
    ContactsWrapper wrapper = msgQueue.front();
    ConstContactsPtr msg = wrapper.msg;
    std::string fingerName = wrapper.name;
    msgQueue.pop();
    int numContacts = msg->contact_size();
    if(numContacts > 0){
      // Calculate contact force
      msgs::Vector3d forceVector = msg->contact(0).wrench(0).body_1_wrench().force();
      float f = math::Vector3(forceVector.x(), forceVector.y(), forceVector.z()).GetLength();
      float colorArray[3];
      float forceRange = this->forceMax - this->forceMin;
      for(int i = 0; i < 3; i++){
        float colorRange = this->colorMax[i] - this->colorMin[i];
        colorArray[i] = colorRange/forceRange*f + colorMin[i];
        if(colorArray[i] > colorMin[i]){
          colorArray[i] = colorMin[i];
        } else if (colorArray[i] < colorMax[i]){

          colorArray[i] = colorMax[i];
        }
      }
      QBrush color(QColor(colorArray[0], colorArray[1], colorArray[2]));
      
      this->contactGraphicsItems[fingerName]->setBrush(color);
      this->contactGraphicsItems[fingerName]->setPen(QPen(QColor(0, 0, 0, 0)));

    }

  }

}

////////////////////////////////////////////////

void GUIAratPlugin::PublishTaskMessage(std::string task_name)
{
  msgs::GzString msg;
  msg.set_data(task_name);
  this->taskPub->Publish(msg);

}

void GUIAratPlugin::OnResetClicked(){
  // Signal to the ArrangePlugin to set up the current task
  PublishTaskMessage(this->taskList[currentTaskIndex]);
}

void GUIAratPlugin::OnNextClicked(){
  this->currentTaskIndex = (this->currentTaskIndex+1) % this->taskList.size();
  PublishTaskMessage(this->taskList[currentTaskIndex]);
  this->instructionsView->setDocument(this->instructionsList[currentTaskIndex]);
}

void GUIAratPlugin::OnTaskSent(std::string id, QTextDocument* instructions, int index)
{
  // Show the instructions to the user
  this->instructionsView->setDocument(instructions);
  PublishTaskMessage(id);
  this->currentTaskIndex = index;
}
