/*
 * copyright (C) 2014 Open Source Robotics Foundation
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


#ifndef  _REST_LOGIN_DIALOG_HH_
#define  _REST_LOGIN_DIALOG_HH_

#include <gazebo/gui/qt.h>
#include "gazebo/util/system.hh"

namespace gazebo
{

  namespace gui
  {

    class RestUiWidget;

    class GAZEBO_VISIBLE RestUiLoginDialog : public QDialog
    {
      Q_OBJECT
        
      /// \brief Consotructor
      /// \param[in] _parent Parent QWidget
      /// \param[in] _title the dialog window title bar text
      /// \param[in] _urlLabel the title of the url (ex: super webservice url)
      /// \param[in] _default url text for the url (ex: https://superweb.com)
      public: RestUiLoginDialog(QWidget *_parent,
                              const char* _title,
                              const char* _urlLabel,
                              const char* _defautlUrl);

      /// \brief emitted when the user presses the login button
      signals: void acceptLogin(QString &url, QString& _username, QString& _password);

      /// slot for the AcceptLogin 
      public slots: void slotAcceptLogin();

      /// \brief A label for the url component
      private: QLabel *labelUrl;

      /// \brief A label for the username component
      private: QLabel *labelUsername;

      /// \brief A label for the password
      private: QLabel *labelPassword;

      /// \brief A text field for the default url
      private: QLineEdit* editUrl;

      /// \brief A text field to enter the user name
      private: QLineEdit* editUsername;

      /// \brief A text field to enter the password
      private: QLineEdit* editPassword;

      /// \brief A label to displau errors and information
      private: QLabel *labelInfo;

      /// \brief The standard dialog buttons
      private: QDialogButtonBox *buttons;
      
      /// \brief Getter for User name (of the basic auth REST service)
      public: std::string getUsername() {return username;}

      /// \brief Getter for the password
      public: std::string getPassword() {return password;}

      /// \brief Getter for the Url (https)
      public: std::string getUrl() {return url;}

      private:
        /// \brief login information
        std::string username;
        std::string password;
        std::string url;
    };
  }
}

#endif
