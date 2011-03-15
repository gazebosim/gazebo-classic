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
#ifndef POINTLIGHTMAKER_HH
#define POINTLIGHTMAKER_HH

#include "common/Messages.hh"
#include "gui/EntityMaker.hh"

namespace gazebo
{
	namespace gui
  {
    class PointLightMaker : public EntityMaker
    {
      public: PointLightMaker();
      public: virtual ~PointLightMaker();
    
      public: virtual void Start(Scene *scene);
      public: virtual void Stop();
      public: virtual bool IsActive() const;
  
      public: virtual void MousePushCB(const MouseEvent &event);
      public: virtual void MouseReleaseCB(const MouseEvent &event);
      public: virtual void MouseDragCB(const MouseEvent &event);
    
      private: virtual void CreateTheEntity();
      private: int state;
      private: msgs::Light msg;
      private: std::string lightName;
      private: static unsigned int counter;
    };
  }
}
#endif
