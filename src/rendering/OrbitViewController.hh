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
#ifndef ORBITVIEWCONTROLLER_HH
#define ORBITVIEWCONTROLLER_HH

#include "rendering/ViewController.hh"
#include "math/Vector3.hh"

namespace gazebo
{
	namespace rendering
  {
    class Visual;
  
    class OrbitViewController : public ViewController
    {
      /// \brief Constructor
      public: OrbitViewController(UserCamera *camera);
  
      /// \brief Destructor
      public: virtual ~OrbitViewController();
  
      /// \brief Update
      public: virtual void Update();
  
      /// \brief Handle a mouse event
      public: virtual void HandleMouseEvent(const common::MouseEvent &event);
  
      /// \brief Get the type name of this view controller
      public: static std::string GetTypeString() {return "OrbitViewController";}
  
      /// \brief Translate the focal point
      private: void Translate(math::Vector3 vec);
  
      /// \brief Normalize yaw value
      private: void NormalizeYaw(float &v);
  
      /// \brief Normalize pitch value
      private: void NormalizePitch(float &v);
  
      private: float yaw, pitch;
      private: float distance;
      private: math::Vector3 focalPoint;
  
      private: Visual *refVisual;
    };
  }

}
#endif
