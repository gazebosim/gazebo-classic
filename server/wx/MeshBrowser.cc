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
#include "Visual.hh"
#include "Simulator.hh"
#include "MeshBrowser.hh"

using namespace gazebo;

class MeshNameTreeItemData : public wxTreeItemData
{
  public: std::string filename;
};


MeshBrowser::MeshBrowser(wxWindow *parent)
  : wxFrame(parent, wxID_ANY, wxT("Mesh Browser"), wxDefaultPosition, wxSize(600, 600), wxDEFAULT_FRAME_STYLE)
{
  wxBoxSizer *boxSizer1 = new wxBoxSizer(wxHORIZONTAL);

  this->treeCtrl = new wxTreeCtrl(this, wxID_ANY, wxDefaultPosition, wxDefaultSize);
  this->treeCtrl->Connect(wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler(MeshBrowser::OnTreeClick), NULL, this);
  boxSizer1->Add(this->treeCtrl,1, wxALL | wxEXPAND| wxALIGN_CENTER_VERTICAL, 5); 

  this->scene = new Scene("viewer_scene");
  this->scene->SetType(Scene::GENERIC);
  this->scene->SetAmbientColor(Color(0.5, 0.5, 0.5));
  this->scene->SetBackgroundColor(Color(0.5, 0.5, 0.5, 1.0));
  this->scene->CreateGrid( 10, 1, 0.03, Color(1,1,1,1));
  this->scene->Init();

  this->dirLight = new Light(NULL, scene);
  this->dirLight->Load(NULL);
  this->dirLight->SetLightType("directional");
  this->dirLight->SetDiffuseColor( Color(1.0, 1.0, 1.0) );
  this->dirLight->SetSpecularColor( Color(0.1, 0.1, 0.1) );
  this->dirLight->SetDirection( Vector3(0, 0, -1) );


  this->renderControl = new RenderControl(this);
  this->renderControl->ViewScene(this->scene);
  this->renderControl->Init();
  UserCamera *cam = this->renderControl->GetCamera();
  cam->SetClipDist(0.01, 1000);
  cam->SetWorldPosition(Vector3(-1,0,2));
  cam->RotatePitch(DTOR(-30));
  cam->SetViewController( OrbitViewController::GetTypeString() );
  boxSizer1->Add(this->renderControl,2, wxALL | wxEXPAND | wxALIGN_CENTER_VERTICAL, 5); 

  GazeboConfig *config = Simulator::Instance()->GetGazeboConfig();
  std::list<std::string> ogrePaths = config->GetGazeboPaths();
  std::list<std::string>::iterator iter;

  for (iter = ogrePaths.begin(); iter != ogrePaths.end(); iter++)
  {
    std::string path(*iter);
    path += "/Media/models";

    DIR *dir=opendir(path.c_str()); 
    if (dir == NULL)
      continue;
    closedir(dir);

    wxString wxpath(path.c_str(), wxConvUTF8);

    wxTreeItemId rootId = this->treeCtrl->AddRoot(wxpath,-1,-1,NULL);
    this->ParseDir(path, rootId);
  }


  this->SetSizer(boxSizer1);
  this->Layout();

  this->visual = NULL;

}

MeshBrowser::~MeshBrowser()
{
  delete this->renderControl;
  delete this->dirLight;
  delete this->scene;
}

int MeshBrowser::ParseDir(const std::string &path, wxTreeItemId &parentId)
{
  int itemCount = 0;
  struct dirent *dir_entry;
  DIR *dir = opendir(path.c_str()); 
  if (dir == NULL)
    return 0;

  while ( (dir_entry = readdir(dir)) != NULL )
  {
    std::string filename = dir_entry->d_name;

    if (filename == "." || filename == "..")
      continue;


    if (dir_entry->d_type == DT_DIR)
    {
      wxTreeItemId itemId = this->treeCtrl->AppendItem(parentId, wxString::FromAscii(filename.c_str()), -1, -1, NULL);
      itemCount += this->ParseDir( path + "/" + filename, itemId );

      if (itemCount == 0)
        this->treeCtrl->Delete(itemId);
    }
    else if (MeshManager::Instance()->IsValidFilename( filename ) )
    {
      itemCount++;
      MeshNameTreeItemData *data = new MeshNameTreeItemData;
      data->filename =  path + "/" + filename.c_str();

      wxTreeItemId itemId = this->treeCtrl->AppendItem(parentId, wxString::FromAscii(filename.c_str()), -1, -1, data);
    }
  }

  closedir(dir);
  return itemCount;
}

void MeshBrowser::OnTreeClick(wxTreeEvent &event)
{
  MeshNameTreeItemData *data = (MeshNameTreeItemData*)this->treeCtrl->GetItemData(event.GetItem());

  if (data == NULL)
    return;

  const Mesh *mesh = MeshManager::Instance()->Load(data->filename);
  OgreCreator::Instance()->InsertMesh(mesh);

  if (this->visual)
    delete this->visual;
  this->visual = NULL;

  this->visual = new Visual(data->filename + "_VISUAL", this->scene);

  try
  {
  this->visual->AttachMesh( data->filename );
  } 
  catch (...)
  {
    printf("Unable to load mesh\n");
  }

  this->visual->SetCastShadows(false);
  this->visual->SetUseRTShader(false);

  if ( !this->visual->HasMaterial() )
    this->visual->SetMaterial("Gazebo/Viewer");

  Vector3 size = this->visual->GetBoundingBoxSize();
  if (size.x > 2.0 || size.y > 2.0 || size.z > 2.0)
  {
    double max = std::max(size.x,size.y);
    max = std::max(max, size.z);
    this->visual->SetScale(Vector3(2.0/max, 2.0/max, 2.0/max ));
  }

  std::cout << data->filename << "\n";
}
