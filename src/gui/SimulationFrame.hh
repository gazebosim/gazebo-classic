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
#ifndef SIMULATIONFRAME_HH
#define SIMULATIONFRAME_HH

#include <wx/wx.h>
#include <wx/treectrl.h>

#include "common/Event.hh"

class wxAuiManager;
class wxAuiManagerEvent;
class wxPropertyGrid;
class wxPropertyGridEvent;

namespace gazebo
{
  namespace common
  {
    class Common;
  }

	namespace gui
  {
    class RenderPanel;
    class TimePanel;
    class DiagnosticsDialog;
  
    class SimulationFrame : public wxFrame
    {
      enum ToolbarButtons {PLAY, PAUSE, STEP, BOX, SPHERE, CYLINDER, DIRECTIONAL, POINT, SPOT, CURSOR};
      enum MenuIds {ID_OPEN, ID_LOAD_MESH, ID_SAVE, ID_RESET, ID_WIREFRAME, ID_PHYSICS, ID_BOUNDING, ID_JOINTS, ID_CONTACTS, ID_LIGHTS, ID_CAMERAS, ID_SNAPTOGRID, ID_EDITGRID};
  
      public: SimulationFrame(wxWindow *parent);
  
      public: virtual ~SimulationFrame();
  
      /// \brief Create the cameras
      public: void ViewScene(rendering::Scene *scene);
  
      public: void Init();
  
      public: void Update();
  
      private: void OnPause(bool pause);
  
      private: void OnQuit(wxCommandEvent &event);
      private: void Quit();
      private: void OnOpen(wxCommandEvent &event);
      private: void OnSave(wxCommandEvent &event);
      private: void OnReset(wxCommandEvent &event);
      private: void OnLoadMesh(wxCommandEvent & WXUNUSED(event));
  
      private: void OnSnapToGrid(wxCommandEvent &event);
      private: void OnEditGrid(wxCommandEvent &event);
      private: void OnCreateModel(wxCommandEvent &event);
  
      private: void OnWireframe(wxCommandEvent &event);
      private: void OnShowPhysics(wxCommandEvent &event);
      private: void OnShowBoundingBoxes(wxCommandEvent &event);
      private: void OnShowJoints(wxCommandEvent &event);
      private: void OnShowContacts(wxCommandEvent &event);
      private: void OnShowLights(wxCommandEvent &event);
      private: void OnShowCameras(wxCommandEvent &event);
      private: void OnDiagnostics(wxCommandEvent &event);
  
      private: void OnToolClicked( wxCommandEvent &event );
  
      private: void OnPaneClosed(wxAuiManagerEvent &event);
  
      private: void OnTreeClick(wxTreeEvent &event);
  
      /// \brief Callback when an entity in the tree is clicked
      private: void OnTreeRightClick(wxTreeEvent &event);
  
      /// \brief When a popup menu item has been click in the tree widget
      private: void OnTreePopupClick( wxCommandEvent &event );
  
      private: void OnPropertyChanged(wxPropertyGridEvent &event);
  
      /// \brief Make the toolbar
      private: void MakeToolbar();
  
      /// \brief Add entity CB
      private: void AddEntityCB(const std::string &name);
  
      /// \brief Delete entity CB
      private: void DeleteEntityCB(const std::string &name);
  
      private: void SetSelectedEntityCB(const std::string &name);
  
      private: void MoveModeCB(const bool &mode);
  
      /// \brief Find an item in a tree
      private: wxTreeItemId FindTreeItem(const std::string &name);
  
      private: void AddEntityHelper(const common::Common *entity, wxTreeItemId parent);
  
      private: RenderPanel *renderPanel;
      private: TimePanel *timePanel;
  
      private: wxAuiManager *auiManager;
      private: wxToolBar *toolbar;
  
      private: wxTreeCtrl *treeCtrl;
      private: wxPropertyGrid *propGrid;
  
      private: DiagnosticsDialog *diagnostics;
      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif
