/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _MOOCUI_WIDGET_HH_
#define _MOOCUI_WIDGET_HH_

#include <gazebo/gazebo.hh>
#include "MOOCLoginDialog.hh"

//typedef const boost::shared_ptr<const ::msgs::RestError> ConstRestErrorPtr;

namespace gazebo
{
  class MOOCUIWidget : public QWidget
  {
    Q_OBJECT

    /// \brief ctor
    public: MOOCUIWidget(QWidget *_parent = 0);
    /// \brief dtor
    public: virtual ~MOOCUIWidget();

    /// \brief QT callback (MOOC/Login menu) 
    public slots: void LoginMOOC();

    /// \brief pub/sub node to communicate with gzserver
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo topics publisher
    private: gazebo::transport::PublisherPtr pub;

    // \brief Gazebo topics subscriber
    private: gazebo::transport::SubscriberPtr sub;

    /// \brief called everytime a response  message is received.
    private: void OnResponse(ConstRestErrorPtr &_msg);   

    /// \brief called before rendering, from the GUI thread
    /// this is called from the plugin's update
    public: void Update();
 
    /// \brief login dialog
    private: gui::MOOCLoginDialog dialog;
    
   private: std::list< boost::shared_ptr<const gazebo::msgs::RestError> > msgRespQ;

 };
}

#endif
