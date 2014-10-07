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

/////////////////////////////////////////////////
GUIAratPlugin::GUIAratPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Next Test"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Create a QGraphicsView to draw the finger force contacts
  this->handScene = new QGraphicsScene(QRectF(0, 0, handImgX, handImgY));
  QGraphicsView *handView = new QGraphicsView(handScene);

  // Add the GraphicsView to the layout
  frameLayout->addWidget(handView);

  // Load the hand image
  QPixmap* handImg = new QPixmap(QString(handImgFilename));
  QGraphicsPixmapItem* handItem = new QGraphicsPixmapItem(*handImg);

  // Draw the hand on the canvas
  handScene->addItem(handItem);
  handScene->update();
  handView->show();

  // Add the button to the frame's layout
  //frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(0, 0);
  this->resize(handImgX+10, handImgY+10);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->taskPub = this->node->Advertise<msgs::GzString>("/gazebo/arat/control");
  this->taskNum = 0;
  this->maxTaskCount = 10;

  //Parse the finger names and point where we will draw a contact
  
  std::ifstream fileinput(this->fingerPtsFilename);
  std::stringstream inputStream;
  inputStream << fileinput.rdbuf();
  char buffer[256];
  inputStream.getline(buffer, 256);
  while(!inputStream.eof() && !inputStream.fail()){
    std::stringstream ss;
    ss << buffer;
    std::vector<std::string> tokens;
    for(int i = 0; i < 3; i++){
      char token[256];
      ss.getline(token, 256, ',');
      //std::getline(ss, token, ',');
      tokens.push_back(std::string(token));
    }
    this->finger_points[tokens[0]] = std::pair<int, int>(atoi(tokens[1].c_str()), atoi(tokens[2].c_str())) ;
    inputStream.getline(buffer, 256);
  }

  fileinput.close();

  // Preallocate some QGraphicsItems
  for(int i = 0; i < 5; i++){
    int xpos = finger_points[fingerNames[i]].first;
    int ypos = finger_points[fingerNames[i]].second;
    this->contactGraphicsItems[fingerNames[i]] = new QGraphicsEllipseItem(xpos, ypos, circleSize, circleSize);
  }


  // Set up an array of subscribers for each contact sensor

  //TODO: set up configurable handedness
  this->handSide = "r";
  
  contactSubscribers.push_back(this->node->Subscribe(this->getTopicName("Th"),
                              &GUIAratPlugin::OnThumbContact, this));
  contactSubscribers.push_back(this->node->Subscribe(this->getTopicName("Ind"),
                              &GUIAratPlugin::OnIndexContact, this));
  contactSubscribers.push_back(this->node->Subscribe(this->getTopicName("Mid"),
                              &GUIAratPlugin::OnMiddleContact, this));
  contactSubscribers.push_back(this->node->Subscribe(this->getTopicName("Ring"),
                              &GUIAratPlugin::OnRingContact, this));
  contactSubscribers.push_back(this->node->Subscribe(this->getTopicName("Little"),
                              &GUIAratPlugin::OnLittleContact, this));

  this->connections.push_back(event::Events::ConnectPreRender(boost::bind(&GUIAratPlugin::PreRender, this)));

}

/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
}

std::string GUIAratPlugin::getTopicName(std::string fingerName){
  std::string topicName = this->handSide+fingerName+"Distal";
  return "/gazebo/default/mpl/"+topicName+"/"+topicName+"_contact_sensor";
}

void GUIAratPlugin::OnThumbContact(ConstContactsPtr &msg){
  this->OnFingerContact(msg, "Th");
}

void GUIAratPlugin::OnIndexContact(ConstContactsPtr &msg){
  this->OnFingerContact(msg, "Ind");
}

void GUIAratPlugin::OnMiddleContact(ConstContactsPtr &msg){
  this->OnFingerContact(msg, "Mid");
}


void GUIAratPlugin::OnRingContact(ConstContactsPtr &msg){
  this->OnFingerContact(msg, "Ring");
}


void GUIAratPlugin::OnLittleContact(ConstContactsPtr &msg){
  this->OnFingerContact(msg, "Little");
}


void GUIAratPlugin::OnFingerContact(ConstContactsPtr &msg, std::string fingerName){
  this->msgQueue.push(ContactsWrapper(msg, fingerName));
}

void GUIAratPlugin::PreRender(){
  for(int i = 0; i < 5; i ++){
    std::string fingerName = this->fingerNames[i];

    if(this->handScene->items().contains(this->contactGraphicsItems[fingerName])){
      this->handScene->removeItem(this->contactGraphicsItems[fingerName]);
    }
  }
  //TODO: color, position offset

  // Parse the contact object and get the topic name
  while( !this->msgQueue.empty()){
    ContactsWrapper wrapper = msgQueue.front();
    ConstContactsPtr msg = wrapper.msg;
    std::string fingerName = wrapper.name;
    msgQueue.pop();
    int numContacts = msg->contact_size();
    if(numContacts > 0){
        
      // Draw on the corresponding spot
      if(!this->handScene->items().contains(this->contactGraphicsItems[fingerName])){
        this->handScene->addItem(this->contactGraphicsItems[fingerName]);
      }
    }

  }

  //Clear the queue
  /**/
}

/////////////////////////////////////////////////
void GUIAratPlugin::OnButton()
{
  this->taskNum = (this->taskNum + 1) % this->maxTaskCount;

  // Send the model to the gazebo server
  msgs::GzString msg;
  msg.set_data("task" + boost::lexical_cast<std::string>(this->taskNum));
  this->taskPub->Publish(msg);
}
