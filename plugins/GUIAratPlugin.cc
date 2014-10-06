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

  //finger_points = YAML::LoadFile("fingerpts.yaml");
  std::filebuf fb;
  fb.open("fingerpts.csv", std::ios::in);
  std::stringstream inputStream;
  while(!inputStream.eof() && !inputStream.fail()){
    char buffer[256];
    inputStream.getline(buffer, 256);
    std::istringstream ss(std::string(buffer));
    std::vector<std::string> tokens;
    for(int i = 0; i < 3; i++){
      std::string token;
      std::getline(ss, token, ',');
      tokens.push_back(token);
    }
    
    this->finger_points[tokens[0]] = std::pair<int, int>(atoi(tokens[1].c_str()), atoi(tokens[2].c_str())) ;
  }

  // Set up an array of subscribers for each contact sensor

  //TODO: set up configurable handedness
  this->handSide = "r";
  for(int i = 0; i < 6; i++){
    std::string topicName = this->handSide+this->fingerNames[i]+"Distal";
    topicName = "/gazebo/default/mpl/"+topicName+
                            "/"+topicName+"_contact_sensor";
    contactSubscribers.push_back(this->node->Subscribe( topicName,
                                 &GUIAratPlugin::OnFingerContact, this ));
  }

  


}

/////////////////////////////////////////////////
GUIAratPlugin::~GUIAratPlugin()
{
}

void GUIAratPlugin::OnFingerContact(ConstContactsPtr &msg){
  // Lock
  contactLock.lock();
  // Parse the contact object and get the topic name
  int numContacts = msg->contact_size();
  if(numContacts > 0){
    for(int i = 0; i < numContacts; i++){
      std::string contactName = msg->contact(0).collision1();

      std::size_t beginIndex = ("mpl::"+this->handSide).length() - 1;
      std::size_t endIndex = contactName.find_first_of("Distal");

      std::string fingerName = contactName.substr(beginIndex, endIndex - beginIndex );
      
      std::pair<int, int> point = finger_points[fingerName];
        
      std::cout << "Drawing on " << point.first << ", " << point.second << std::endl;

      // Draw on the corresponding spot
      this->handScene->addEllipse(point.first, point.second, circleSize, circleSize);
    }

    this->handScene->update();
  }
  // Unlock
  contactLock.unlock();
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
