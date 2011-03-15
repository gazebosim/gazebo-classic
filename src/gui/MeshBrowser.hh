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
#ifndef MODELBROWSER_HH
#define MODELBROWSER_HH

#include <wx/wx.h>
#include <wx/treectrl.h>

namespace gazebo
{
	namespace gui
  {
    class RenderControl;
  
    class MeshBrowser : public wxFrame
    {
      public: MeshBrowser(wxWindow *parent);
  
      public: virtual ~MeshBrowser();
  
      private: int ParseDir(const std::string &path, wxTreeItemId &parentId);
      private: void OnTreeClick(wxTreeEvent &event);
  
      private: wxTreeCtrl *treeCtrl;
  
      private: RenderControl *renderControl;
  
      //private: Visual *visual;
      //private: Scene *scene;
      //private: Light *dirLight;
    };
  }

}
#endif
