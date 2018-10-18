#include <wx/aui/aui.h>
#include <stack>

#include "propgrid/propgrid.h"

#include "ParamsNotebook.hh"

#include "MeshBrowser.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "XMLConfig.hh"
#include "OgreCamera.hh"
#include "CameraManager.hh"
#include "World.hh"
#include "Entity.hh"
#include "EntityMaker.hh"
#include "Model.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Events.hh"
#include "Image.hh"
#include "RenderPanel.hh"
#include "TimePanel.hh"
#include "SimulationFrame.hh"
#include "Simulator.hh"

using namespace gazebo;


class EntityTreeItemData : public wxTreeItemData
{
  public: std::string name;
};

////////////////////////////////////////////////////////////////////////////////
// Constructor
SimulationFrame::SimulationFrame(wxWindow *parent)
  : wxFrame(parent, wxID_ANY, wxT("Gazebo"), wxDefaultPosition, wxSize(1024, 768), wxDEFAULT_FRAME_STYLE)
{
  wxInitAllImageHandlers();

  wxMenuBar *menuBar = new wxMenuBar;
  wxMenu *fileMenu = new wxMenu;
  wxMenu *editMenu = new wxMenu;
  wxMenu *viewMenu = new wxMenu;

  wxMenuItem *openItem = fileMenu->Append(ID_OPEN, wxT("&Open\tCtrl-O") );
  Connect(openItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnOpen), NULL, this);

  wxMenuItem *loadMeshItem = fileMenu->Append(ID_LOAD_MESH, wxT("&Load Mesh\tCtrl-M") );
  Connect(loadMeshItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnLoadMesh), NULL, this);


  wxMenuItem *saveItem = fileMenu->Append(ID_SAVE, wxT("&Save\tCtrl-S") );
  Connect(saveItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnSave), NULL, this);

  wxMenuItem *resetItem = fileMenu->Append(ID_RESET, wxT("&Reset\tCtrl-R") );
  Connect(resetItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnReset), NULL, this);

  wxMenuItem *quitItem = fileMenu->Append(wxID_EXIT, wxT("&Quit\tCtrl-Q") );
  Connect(quitItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnQuit), NULL, this);


  wxMenuItem *snapToGridItem = editMenu->AppendCheckItem( ID_SNAPTOGRID, wxT("Snap To Grid"));
  snapToGridItem->Check();
  Connect(snapToGridItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnSnapToGrid), NULL, this);

  wxMenuItem *editGridItem = editMenu->Append( ID_EDITGRID, wxT("Edit Grid"));
  Connect(editGridItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnEditGrid), NULL, this);


  wxMenuItem *wireItem = viewMenu->AppendCheckItem(ID_WIREFRAME, wxT("Wireframe"));
  Connect(wireItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnWireframe), NULL, this);

  wxMenuItem *showPhysicsItem= viewMenu->AppendCheckItem(ID_PHYSICS, wxT("Show Physics"));
  Connect(showPhysicsItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowPhysics), NULL, this);

  wxMenuItem *showBoundingBoxesItem= viewMenu->AppendCheckItem(ID_BOUNDING, wxT("Show Bounding Boxes"));
  Connect(showBoundingBoxesItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowBoundingBoxes), NULL, this);

  wxMenuItem *showJointsItem= viewMenu->AppendCheckItem(ID_JOINTS, wxT("Show Joints"));
  Connect(showJointsItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowJoints), NULL, this);

  wxMenuItem *showContactsItem= viewMenu->AppendCheckItem(ID_CONTACTS, wxT("Show Contacts"));
  Connect(showContactsItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowContacts), NULL, this);

  wxMenuItem *showLightsItem= viewMenu->AppendCheckItem(ID_LIGHTS, wxT("Show Lights"));
  Connect(showLightsItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowLights), NULL, this);

  wxMenuItem *showCamerasItem= viewMenu->AppendCheckItem(ID_CAMERAS, wxT("Show Cameras"));
  Connect(showCamerasItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(SimulationFrame::OnShowCameras), NULL, this);


  menuBar->Append( fileMenu, _("&File") );
  menuBar->Append( editMenu, _("&Edit") );
  menuBar->Append( viewMenu, _("&View") );
  SetMenuBar( menuBar );

  this->toolbar = NULL;
  this->renderPanel = new RenderPanel(this);
  this->timePanel = new TimePanel(this);
  this->treeCtrl = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize);
  this->propGrid = new wxPropertyGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_DEFAULT_STYLE|wxPG_SPLITTER_AUTO_CENTER);
  this->propGrid->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( SimulationFrame::OnPropertyChanged), NULL, this);

  Connect(this->treeCtrl->GetId(), wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler(SimulationFrame::OnTreeClick), NULL, this); 
  this->treeCtrl->Connect(wxEVT_COMMAND_TREE_ITEM_RIGHT_CLICK, wxTreeEventHandler(SimulationFrame::OnTreeRightClick), NULL, this);


  this->MakeToolbar();

  this->auiManager = new wxAuiManager(this);
  this->auiManager->AddPane(this->renderPanel, wxAuiPaneInfo().CenterPane().Name(wxT("Render")));
  this->auiManager->AddPane(this->timePanel, wxAuiPaneInfo().RightDockable(false).LeftDockable(false).Bottom().Name(wxT("Time")).Caption(wxT("Time")));
  this->auiManager->AddPane(this->propGrid, wxAuiPaneInfo().BestSize(230,200).Left().Name(wxT("Properties")).Caption(wxT("Properties")));
  this->auiManager->AddPane(this->treeCtrl, wxAuiPaneInfo().BestSize(230,200).Left().Name(wxT("Entities")).Caption(wxT("Entities")));

  if (this->toolbar)
    this->auiManager->AddPane(this->toolbar, wxAuiPaneInfo().ToolbarPane().RightDockable(false).LeftDockable(false).MinSize(100,30).Top().Name(wxT("Tools")).Caption(wxT("Tools")));
  this->auiManager->Update();

  Connect(wxEVT_AUI_PANE_CLOSE, wxAuiManagerEventHandler(SimulationFrame::OnPaneClosed), NULL, this);


  Simulator::Instance()->ConnectPauseSignal( 
      boost::bind(&SimulationFrame::OnPause, this, _1) );

  Events::ConnectAddEntitySignal( 
      boost::bind(&SimulationFrame::AddEntityCB, this, _1) );

  Events::ConnectDeleteEntitySignal( 
      boost::bind(&SimulationFrame::DeleteEntityCB, this, _1) );

  Events::ConnectSetSelectedEntitySignal(
     boost::bind(&SimulationFrame::SetSelectedEntityCB, this, _1) );

  Events::ConnectMoveModeSignal( 
      boost::bind(&SimulationFrame::MoveModeCB, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SimulationFrame::~SimulationFrame()
{
  this->propGrid->Destroy();
  this->auiManager->UnInit();
  delete this->auiManager;
}

////////////////////////////////////////////////////////////////////////////////
// Create the cameras
void SimulationFrame::CreateCameras()
{
  this->renderPanel->CreateCamera(0);
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::Init()
{
  this->renderPanel->Init();
  this->OnPause(Simulator::Instance()->IsPaused());

  EntityTreeItemData *data;

  data = new EntityTreeItemData;
  data->name = "World";
  wxTreeItemId rootId = this->treeCtrl->AddRoot(wxT("World"),-1,-1,data);
}

////////////////////////////////////////////////////////////////////////////////
// Add entity CB
void SimulationFrame::AddEntityCB(const std::string &name)
{
  /*EntityTreeItemData *data;

  data = new EntityTreeItemData;
  data->name = entity->GetCompleteScopedName();

  wxTreeItemId item = this->treeCtrl->AppendItem(this->treeCtrl->GetRootItem(), wxString::FromAscii(entity->GetName().c_str()), -1, -1, data);
  */
  Entity *entity = dynamic_cast<Entity*>(Common::GetByName(name));
  this->AddEntityHelper(entity, this->treeCtrl->GetRootItem());
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::AddEntityHelper(const Common *entity, wxTreeItemId parent)
{
  EntityTreeItemData *data;
  wxTreeItemId item;

  data = new EntityTreeItemData;
  data->name = entity->GetCompleteScopedName();

  //if (entity->GetName() != "COM_Entity")
  if (entity->GetShowInGui())
    item = this->treeCtrl->AppendItem(parent, wxString::FromAscii(entity->GetName().c_str()), -1, -1, data);
  else
    item = parent;

  for (unsigned int i=0; i < entity->GetChildCount(); i++)
  {
    Common *childEntity = entity->GetChild(i);
    if (childEntity)
    {
      this->AddEntityHelper(childEntity, item);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Delete entity callback
void SimulationFrame::DeleteEntityCB(const std::string &name)
{
  this->treeCtrl->Delete( this->FindTreeItem(name) );
}

////////////////////////////////////////////////////////////////////////////////
// Find an item in a tree
wxTreeItemId SimulationFrame::FindTreeItem(const std::string &name)
{
  std::stack<wxTreeItemId> items;

  if (this->treeCtrl->GetRootItem().IsOk())
    items.push(this->treeCtrl->GetRootItem());

  while (!items.empty())
  {
    wxTreeItemId next = items.top();
    items.pop();

    if (next != this->treeCtrl->GetRootItem() && 
       ((EntityTreeItemData*)this->treeCtrl->GetItemData(next))->name == name)
      return next;

    wxTreeItemIdValue cookie;
    wxTreeItemId nextChild = this->treeCtrl->GetFirstChild(next, cookie);
    while (nextChild.IsOk())
    {
      items.push(nextChild);
      nextChild = this->treeCtrl->GetNextSibling(nextChild);
    }
  }

  return wxTreeItemId();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnPause(bool pause)
{
  if (pause)
  {
    this->toolbar->ToggleTool(PLAY, false);
    this->toolbar->ToggleTool(PAUSE, true);
  }
  else
  {
    this->toolbar->ToggleTool(PLAY, true);
    this->toolbar->ToggleTool(PAUSE, false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the frame
void SimulationFrame::Update()
{
  this->timePanel->Update();
  this->renderPanel->Update();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnQuit(wxCommandEvent &event)
{
  Simulator::Instance()->SetUserQuit();
  Close(true);
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnOpen(wxCommandEvent & WXUNUSED(event))
{
  wxFileDialog* openDialog = new wxFileDialog(
      this, _("Choose a file to open"), wxEmptyString, wxEmptyString, 
      _("world files (*.world; *.xml)|*.world;*.xml"), wxFD_OPEN, wxDefaultPosition);

  // Creates a "open file" dialog with 4 file types
  if (openDialog->ShowModal() == wxID_OK) 
  {
    std::string doc( openDialog->GetPath().mb_str() );

    // Load the world file
    XMLConfig *xmlFile = new gazebo::XMLConfig();

    try
    {
      xmlFile->Load(doc);
    }
    catch (GazeboError e)
    {
      gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
    }

    //XMLConfigNode *rootNode(xmlFile->GetRootNode());
    //Create the world
    try
    {
      Simulator::Instance()->StopPhysics();
      //World::Instance()->Clear();
      Simulator::Instance()->Load( doc, World::Instance()->GetServerId() );
      Simulator::Instance()->StartPhysics();
    }
    catch (GazeboError e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }
  }

  // Clean up after ourselves
  openDialog->Destroy();
}

////////////////////////////////////////////////////////////////////////////////
// On load mesh
void SimulationFrame::OnLoadMesh(wxCommandEvent & WXUNUSED(event))
{
  MeshBrowser *browser = new MeshBrowser(this);
  browser->Show();


}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnSave(wxCommandEvent & WXUNUSED(event))
{
  wxFileDialog* saveDialog = new wxFileDialog(
      this, _("Choose a file to save"), wxEmptyString, wxEmptyString, 
      _("world files (*.world; *.xml)|*.world;*.xml"), wxFD_SAVE, wxDefaultPosition);

  // Creates a "open file" dialog with 4 file types
  if (saveDialog->ShowModal() == wxID_OK) 
  {
    std::string doc( saveDialog->GetPath().mb_str() );

    Simulator::Instance()->Save( doc );
  }

}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnReset(wxCommandEvent & WXUNUSED(event))
{
  // stop simulation when this is happening
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
  World::Instance()->Reset();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnSnapToGrid(wxCommandEvent &event)
{
  EntityMaker::SetSnapToGrid( event.IsChecked() );
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnEditGrid(wxCommandEvent &event)
{
  ParamsNotebook notebook(this);
  if (notebook.ShowModal() == wxID_OK)
    printf("Good\n");
  else
    printf("Bad\n");
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnWireframe(wxCommandEvent &event)
{
  Events::wireframeSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowPhysics(wxCommandEvent &event)
{
  Events::showPhysicsSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowBoundingBoxes(wxCommandEvent &event)
{
  Events::showBoundingBoxesSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowJoints(wxCommandEvent &event)
{
  Events::showJointsSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowContacts(wxCommandEvent &event)
{
  Events::showContactsSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowLights(wxCommandEvent &event)
{
  Events::showLightsSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnShowCameras(wxCommandEvent &event)
{
  Events::showCamerasSignal();
}

////////////////////////////////////////////////////////////////////////////////
void SimulationFrame::OnToolClicked( wxCommandEvent &event )
{
  int id = event.GetId();

  this->toolbar->ToggleTool(BOX, false);
  this->toolbar->ToggleTool(SPHERE, false);
  this->toolbar->ToggleTool(CYLINDER, false);
  this->toolbar->ToggleTool(SPOT, false);
  this->toolbar->ToggleTool(POINT, false);
  this->toolbar->ToggleTool(DIRECTIONAL, false);


  if (id == PLAY)
  {
    this->toolbar->ToggleTool(PAUSE, false);
    this->toolbar->ToggleTool(STEP, false);
    Simulator::Instance()->SetPaused(false);
  }
  else if (id == PAUSE)
  {
    this->toolbar->ToggleTool(PLAY, false);
    this->toolbar->ToggleTool(STEP, false);
    Simulator::Instance()->SetPaused(true);
  }
  else if (id == STEP)
  {
    this->toolbar->ToggleTool(PLAY, false);
    this->toolbar->ToggleTool(STEP, false);
    Simulator::Instance()->SetStepInc( true );
  }
  else if (id == BOX)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("box");
  }
  else if (id == SPHERE)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("sphere");
  }
  else if (id == CYLINDER)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("cylinder");
  }
  else if (id == POINT)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("pointlight");
  }
  else if (id == SPOT)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("spotlight");
  }
  else if (id == DIRECTIONAL)
  {
    this->renderPanel->SetCursor(*wxCROSS_CURSOR);
    Events::createEntitySignal("directionallight");
  }
  else if (id == CURSOR)
  {
    this->renderPanel->SetCursor(*wxSTANDARD_CURSOR);
    Events::createEntitySignal("");
  }
}


void SimulationFrame::OnPaneClosed(wxAuiManagerEvent &event)
{
  this->auiManager->Update();
  //wxAuiPaneInfo* pane = event.GetPane();
  //wxWindow* window = pane->window;
  //menubar_->Check(window->GetId(), false);
}

////////////////////////////////////////////////////////////////////////////////
// Callback when an entity in the tree is clicked
void SimulationFrame::OnTreeClick(wxTreeEvent &event)
{
  EntityTreeItemData *data = (EntityTreeItemData*)this->treeCtrl->GetItemData(event.GetItem());

  if (data)
  {
    this->propGrid->Clear();

    unsigned int paramCount = 0;
    Common *common = NULL;

    if (data->name == "World")
      paramCount = World::Instance()->GetParamCount();
    else 
    {
      common = Common::GetByName( data->name );
      paramCount = common->GetParamCount();
    }

    for (unsigned int i = 0; i < paramCount; i++)
    {
      Param *param = NULL;

      if (data->name == "World")
        param = World::Instance()->GetParam(i);
      else
        param = common->GetParam(i);

      wxString paramName = wxString::FromAscii(param->GetKey().c_str());
      wxString paramValue = wxString::FromAscii(param->GetAsString().c_str());

      wxPGProperty *prop = NULL;
      if (param->IsInt())
        prop = this->propGrid->Append(new wxIntProperty(paramName, wxPG_LABEL));
      else if (param->IsUInt())
        prop = this->propGrid->Append(new wxUIntProperty(paramName, wxPG_LABEL));
      else if (param->IsBool())
      {
        prop = this->propGrid->Append(new wxBoolProperty(paramName, wxPG_LABEL));
        if (param->GetAsString() == "1")
          paramValue = wxT("True");
        else 
          paramValue = wxT("False");
      }
      else if (param->IsFloat() || param->IsDouble())
        prop = this->propGrid->Append(new wxFloatProperty(paramName, wxPG_LABEL));
      else if (param->IsStr())
        prop = this->propGrid->Append(new wxStringProperty(paramName, wxPG_LABEL));

      if (prop)
      {
        prop->SetValueFromString( paramValue );
        prop->SetClientData( param );
      }
    }

    this->propGrid->Refresh();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Callback when an entity in the tree is clicked
void SimulationFrame::OnTreeRightClick(wxTreeEvent &event)
{
  EntityTreeItemData *data = (EntityTreeItemData*)this->treeCtrl->GetItemData(event.GetItem());

  if (data->name != "World")
  {
    wxMenu mnu;
    mnu.Append(0,  wxT("Move to"));
    mnu.Append(1,  wxT("Modify"));
    mnu.Append(2,  wxT("Delete"));
    mnu.Connect(wxEVT_COMMAND_MENU_SELECTED, (wxObjectEventFunction)&SimulationFrame::OnTreePopupClick, NULL, this);
    PopupMenu(&mnu);
  }
}

////////////////////////////////////////////////////////////////////////////////
// When a popup menu item has been click in the tree widget
void SimulationFrame::OnTreePopupClick( wxCommandEvent &event )
{
  EntityTreeItemData *data = (EntityTreeItemData*)this->treeCtrl->GetItemData(this->treeCtrl->GetSelection());

  Common *common = Common::GetByName(data->name);

  if (common && common->HasType(ENTITY))
  {
    Entity *ent = dynamic_cast<Entity*>(common);

    if (event.GetId() == 0)
    {
      OgreCamera *cam = CameraManager::Instance()->GetActiveCamera();
      if (cam)
        cam->MoveToEntity(ent);

      Events::setSelectedEntitySignal("");
      this->toolbar->ToggleTool(CURSOR, true);
    }
    else if (event.GetId() == 1)
      Events::setSelectedEntitySignal(data->name);
    else if (event.GetId() == 2)
    {
      Events::setSelectedEntitySignal("");
      Events::deleteEntitySignal(data->name);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// On property changed event callback
void SimulationFrame::OnPropertyChanged(wxPropertyGridEvent &event)
{
  wxPGProperty *prop = event.GetProperty();
  if (!prop)
    return;

  Param *param = (Param*)(prop->GetClientData());
  if (param)
  {
    std::string value = std::string(prop->GetValueAsString().ToAscii());
    param->SetFromString( value, true );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Make the toolbar
void SimulationFrame::MakeToolbar()
{
#if !defined(__WXMAC__)
  Image image;

  image.Load("control_play_blue.png");
  wxBitmap play_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("control_pause_blue.png");
  wxBitmap pause_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("control_end_blue.png");
  wxBitmap step_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);


  image.Load("sphere_create_blue.png");
  wxBitmap sphere_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("box_create_blue.png");
  wxBitmap box_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("cylinder_create_blue.png");
  wxBitmap cylinder_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("pointlight.png");
  wxBitmap pointlight_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("spotlight.png");
  wxBitmap spotlight_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("directionallight.png");
  wxBitmap directionallight_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  image.Load("cursor.png");
  wxBitmap cursor_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);



  this->toolbar = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTB_HORIZONTAL);
  this->toolbar->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( SimulationFrame::OnToolClicked), NULL , this );

  this->toolbar->AddCheckTool(PLAY, wxT("Play"), play_bitmap, wxNullBitmap, wxT("Play the simulation"));
  this->toolbar->AddCheckTool(PAUSE, wxT("Pause"), pause_bitmap, wxNullBitmap, wxT("Pause the simulation"));
  this->toolbar->AddCheckTool(STEP, wxT("Step"), step_bitmap, wxNullBitmap, wxT("Step the simulation"));
  this->toolbar->AddSeparator();


  this->toolbar->AddRadioTool(CURSOR, wxT("Cursor"), cursor_bitmap, wxNullBitmap, wxT("Camera movement"));
  this->toolbar->AddRadioTool(BOX, wxT("Box"), box_bitmap, wxNullBitmap, wxT("Create a box"));
  this->toolbar->AddRadioTool(SPHERE, wxT("Sphere"), sphere_bitmap, wxNullBitmap, wxT("Create a sphere"));
  this->toolbar->AddRadioTool(CYLINDER, wxT("Cylinder"), cylinder_bitmap, wxNullBitmap, wxT("Create a cylinder"));
  this->toolbar->AddRadioTool(POINT, wxT("Point"), pointlight_bitmap, wxNullBitmap, wxT("Create a point light source"));
  this->toolbar->AddRadioTool(SPOT, wxT("Spot"), spotlight_bitmap, wxNullBitmap, wxT("Create a spot light"));
  this->toolbar->AddRadioTool(DIRECTIONAL, wxT("Directional"), directionallight_bitmap, wxNullBitmap, wxT("Create a directional light"));
  this->toolbar->Realize();

#endif
}

void SimulationFrame::SetSelectedEntityCB(const std::string &name)
{
  if (!name.empty())
    this->toolbar->ToggleTool(CURSOR, true);
}

void SimulationFrame::MoveModeCB(const bool &mode)
{
  if (mode)
  {
    this->toolbar->ToggleTool(CURSOR, true);
    this->toolbar->ToggleTool(BOX, false);
    this->toolbar->ToggleTool(SPHERE, false);
    this->toolbar->ToggleTool(CYLINDER, false);
    this->toolbar->ToggleTool(SPOT, false);
    this->toolbar->ToggleTool(POINT, false);
    this->toolbar->ToggleTool(DIRECTIONAL, false);
  }

}
