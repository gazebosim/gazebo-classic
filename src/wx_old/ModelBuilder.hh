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
#ifndef MODELBUILDER_HH
#define MODELBUILDER_HH

#include <wx/wx.h>

class wxAuiManager;

namespace gazebo
{
	namespace gui
  {
    class RenderControl;
  
    class ModelBuilder : public wxDialog
    {
      enum ToolbarButtons {ADD_BODY};
  
      public: ModelBuilder( wxWindow *parent );
      public: virtual ~ModelBuilder();
  
      private: void MakeToolbar();
      private: void OnToolClicked( wxCommandEvent &event );
  
      private: wxAuiManager *auiManager;
      private: wxToolBar *toolbar;
  
      private: RenderControl *renderControl;
               /*
      private: World *world;
      private: Light *dirLight;
      private: Model *model;
      */
  
    };
  }
}
#endif
