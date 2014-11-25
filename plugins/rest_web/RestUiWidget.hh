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

#ifndef _RestUi_WIDGET_HH_
#define _RestUi_WIDGET_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include "RestUiLoginDialog.hh"

//typedef const boost::shared_ptr<const ::msgs::RestError> ConstRestErrorPtr;

namespace gazebo
{
  class GAZEBO_VISIBLE  RestUiWidget : public QWidget
  {
    Q_OBJECT

    /// \brief ctor
    public: RestUiWidget(QWidget *_parent,
                         const char* _menuTitle,
                         const char* _title,
                         const char* _urlLabel,
                         const char* _defautlUrl
                        );

    /// \brief dtor
    public: virtual ~RestUiWidget();

    /// \brief QT callback (MOOC/Login menu) 
    public slots: void LoginMOOC();

    /// \brief called before rendering, from the GUI thread
    /// this is called from the plugin's update
    public: void Update();

    /// \brief the title to use when displaying dialog/message windows
    private: std::string title;

    /// \brief pub/sub node to communicate with gzserver
    private: gazebo::transport::NodePtr node;

    /// \brief login dialog
    private: gui::RestUiLoginDialog dialog;

    /// \brief Gazebo topics publisher
    private: gazebo::transport::PublisherPtr pub;

    // \brief Gazebo topics subscriber
    private: gazebo::transport::SubscriberPtr sub;

    /// \brief called everytime a response  message is received.
    private: void OnResponse(ConstRestErrorPtr &_msg);   

    /// \brief List of unprocessed error messages to be displayed from the gui thread    
    private: std::list< boost::shared_ptr<const gazebo::msgs::RestError> > msgRespQ;

 };
}

#endif
