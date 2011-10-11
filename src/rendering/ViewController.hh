/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef VIEWCONTROLLER_HH
#define VIEWCONTROLLER_HH

#include "common/CommonTypes.hh"

namespace gazebo
{
	namespace rendering
  {
    class UserCamera;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief Base class for view controllers
    class ViewController
    {
      /// \brief Constructor
      public: ViewController(UserCamera *camera);
  
      /// \brief Destructor
      public: virtual ~ViewController();
  
      public: virtual void Init() = 0;
      public: virtual void Update() = 0;
  
      /// \brief Handle a mouse event
      public: virtual void HandleMouseEvent(const common::MouseEvent &event)=0;

      public: std::string GetTypeString() const;
  
      protected: UserCamera *camera; 
      protected: std::string typeString;
    };
    /// \}
  }
}
#endif
