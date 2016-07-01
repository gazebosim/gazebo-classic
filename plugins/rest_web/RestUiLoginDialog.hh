/*
 * copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _REST_LOGIN_DIALOG_HH_
#define _REST_LOGIN_DIALOG_HH_

#include <string>

#include <gazebo/gui/qt.h>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class RestUiWidget;

    /// \class RestUiLoginDialog RestUiLoginDialog.hh RestUiLoginDialog.hh
    /// \brief Provides a means to login to a webservice
    class GAZEBO_VISIBLE RestUiLoginDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget
      /// \param[in] _title The dialog window title bar text
      /// \param[in] _url Label the title of the url (ex: super webservice url)
      /// \param[in] _defaultUrl Url text for the url (ex: https://superweb.com)
      public: RestUiLoginDialog(QWidget *_parent,
                              const std::string &_title,
                              const std::string &_urlLabel,
                              const std::string &_defautlUrl);

      /// \brief Emitted when the user presses the login button
      /// \param[in] _url The web server url
      /// \param[in] _username The user name
      /// \param[in] _password The user password
      signals: void AcceptLogin(QString &_url,
                                QString &_username,
                                QString &_password);

      /// \brief Getter for User name (of the basic auth REST service)
      /// \return User name
      public: std::string GetUsername() const;

      /// \brief Getter for the password
      /// \return The password
      public: std::string GetPassword() const;

      /// \brief Getter for the Url
      /// \return The url for the site (ex: https://yoursite.com:4000)
      public: std::string GetUrl() const;

      /// \brief Slot for the AcceptLogin event
      public slots: void SlotAcceptLogin();

      /// \brief A label for the url component that appears on the
      /// login widget above
      private: QLabel *labelUrl;

      /// \brief A label for the username component
      private: QLabel *labelUsername;

      /// \brief A label for the password
      private: QLabel *labelPassword;

      /// \brief A text field for the default url
      private: QLineEdit *editUrl;

      /// \brief A text field to enter the user name
      private: QLineEdit *editUsername;

      /// \brief A text field to enter the password
      private: QLineEdit *editPassword;

      /// \brief A label to display errors and information
      private: QLabel *labelInfo;

      /// \brief The standard dialog buttons
      private: QDialogButtonBox *buttons;

      /// \brief The username
      private: std::string username;

      /// \brief The user password
      private: std::string password;

      /// \brief The web server url
      private: std::string url;
    };
  }
}

#endif
