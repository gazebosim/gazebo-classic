#include "OgreAdaptor.hh"
#include "Global.hh"
#include "Light.hh"
#include "GazeboConfig.hh"
#include "UserCamera.hh"
#include "MeshManager.hh"
#include "RenderControl.hh"
#include "OrbitViewController.hh"
#include "FPSViewController.hh"
#include "OgreCreator.hh"
#include "OgreVisual.hh"
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

  this->renderControl = new RenderControl(this);
  this->renderControl->CreateCamera(1);
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
}

void MeshBrowser::ParseDir(const std::string &path, wxTreeItemId &parentId)
{
  std::cout << "ParseDir[" << path << "]\n";

  struct dirent *dir_entry;
  DIR *dir = opendir(path.c_str()); 
  if (dir == NULL)
    return;

  while ( (dir_entry = readdir(dir)) != NULL )
  {
    std::string filename = dir_entry->d_name;

    if (filename == "." || filename == "..")
      continue;


    if (dir_entry->d_type == DT_DIR)
    {
      wxTreeItemId itemId = this->treeCtrl->AppendItem(parentId, wxString::FromAscii(filename.c_str()), -1, -1, NULL);
      this->ParseDir( path + "/" + filename, itemId );
    }
    else
    {
      MeshNameTreeItemData *data = new MeshNameTreeItemData;
      data->filename =  path + "/" + filename.c_str();

      wxTreeItemId itemId = this->treeCtrl->AppendItem(parentId, wxString::FromAscii(filename.c_str()), -1, -1, data);
    }
  }

  closedir(dir);
}

void MeshBrowser::OnTreeClick(wxTreeEvent &event)
{
  MeshNameTreeItemData *data = (MeshNameTreeItemData*)this->treeCtrl->GetItemData(event.GetItem());

  if (data == NULL)
    return;

  const Mesh *mesh = MeshManager::Instance()->Load(data->filename);
  OgreCreator::Instance()->InsertMesh(mesh);

  if (this->visual)
    OgreCreator::Instance()->DeleteVisual(this->visual);

  this->visual = OgreCreator::Instance()->CreateVisual(data->filename + "_VISUAL", NULL, NULL, OgreAdaptor::Instance()->GetScene(1));

  this->visual->AttachMesh( data->filename );

  Vector3 size = this->visual->GetBoundingBoxSize();
  if (size.x > 2.0 || size.y > 2.0 || size.z > 2.0)
  {
    double max = std::max(size.x,size.y);
    max = std::max(max, size.z);
    this->visual->SetScale(Vector3(2.0/max, 2.0/max, 2.0/max ));
  }

  std::cout << data->filename << "\n";
}
