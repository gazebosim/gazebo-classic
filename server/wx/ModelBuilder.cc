#include <wx/aui/aui.h>

#include "Model.hh"
#include "Body.hh"
#include "Image.hh"
#include "OgreAdaptor.hh"
#include "Global.hh"
#include "Light.hh"
#include "Scene.hh"
#include "GazeboConfig.hh"
#include "UserCamera.hh"
#include "MeshManager.hh"
#include "RenderControl.hh"
#include "OrbitViewController.hh"
#include "FPSViewController.hh"
#include "OgreCreator.hh"
#include "OgreVisual.hh"
#include "Simulator.hh"
#include "ModelBuilder.hh"

using namespace gazebo;

ModelBuilder::ModelBuilder( wxWindow *parent )
  : wxDialog(parent,wxID_ANY, wxT("Model Builder"), wxDefaultPosition, wxSize(600, 600), wxDEFAULT_FRAME_STYLE)
{
  this->scene = OgreAdaptor::Instance()->CreateScene("viewer_scene");
  this->dirLight = new Light(NULL, scene);
  this->dirLight->Load(NULL);
  this->dirLight->SetLightType("directional");
  this->dirLight->SetDiffuseColor( Color(1.0, 1.0, 1.0) );
  this->dirLight->SetSpecularColor( Color(0.1, 0.1, 0.1) );
  this->dirLight->SetDirection( Vector3(0, 0, -1) );

  this->renderControl = new RenderControl(this);
  this->renderControl->CreateCamera(this->scene);
  this->renderControl->Init();
  UserCamera *cam = this->renderControl->GetCamera();
  cam->SetClipDist(0.01, 1000);
  cam->SetWorldPosition(Vector3(-1,0,2));
  cam->RotatePitch(DTOR(-30));
  cam->SetViewController( OrbitViewController::GetTypeString() );

  this->MakeToolbar();

  this->auiManager = new wxAuiManager(this);
  this->auiManager->AddPane(this->renderControl, wxAuiPaneInfo().CenterPane().Name(wxT("Render")));
  this->auiManager->AddPane(this->toolbar, wxAuiPaneInfo().ToolbarPane().RightDockable(false).LeftDockable(false).MinSize(100,30).Top().Name(wxT("Tools")).Caption(wxT("Tools")));
  this->auiManager->Update();

  this->model = new Model(NULL);
}

ModelBuilder::~ModelBuilder()
{
  delete this->renderControl;
  delete this->dirLight;
  OgreAdaptor::Instance()->RemoveScene(this->scene->GetName());

  this->auiManager->UnInit();
  delete this->auiManager;

}

void ModelBuilder::MakeToolbar()
{
#if !defined(__WXMAC__)
  Image image;

  image.Load("control_play_blue.png");
  wxBitmap addbody_bitmap(wxString::FromAscii(image.GetFilename().c_str()), wxBITMAP_TYPE_PNG);

  this->toolbar = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTB_HORIZONTAL);
  this->toolbar->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( ModelBuilder::OnToolClicked), NULL , this );

  this->toolbar->AddCheckTool(ADD_BODY, wxT("Add Body"), addbody_bitmap, wxNullBitmap, wxT("Add a body"));

  this->toolbar->Realize();

#endif
}

////////////////////////////////////////////////////////////////////////////////
void ModelBuilder::OnToolClicked( wxCommandEvent &event )
{
  int id = event.GetId();
  if (id == ADD_BODY)
  {
    this->model->CreateBody();
  }
}
