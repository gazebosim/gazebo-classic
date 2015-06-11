/*
 * copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_REST_LOGOUT_DIALOG_HH_
#define _GAZEBO_REST_LOGOUT_DIALOG_HH_

#include <string>

#include <gazebo/gui/qt.h>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Provides a means to logout from a webservice
    class GAZEBO_VISIBLE RestUiLogoutDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget
      /// \param[in] _defaultUrl Url text for the url (ex: https://superweb.com)
      public: RestUiLogoutDialog(QWidget *_parent,
                                 const std::string &_defautlUrl);

      /// \brief Slot for the AcceptLogin event
      public slots: void SlotAcceptLogout();
    };
  }
}

#endif
