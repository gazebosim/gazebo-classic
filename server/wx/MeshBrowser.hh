#ifndef MODELBROWSER_HH
#define MODELBROWSER_HH

#include <wx/wx.h>
#include <wx/treectrl.h>

namespace gazebo
{
  class RenderControl;
  class OgreVisual;
  class Scene;
  class Light;

  class MeshBrowser : public wxFrame
  {
    public: MeshBrowser(wxWindow *parent);

    public: virtual ~MeshBrowser();

    private: int ParseDir(const std::string &path, wxTreeItemId &parentId);
    private: void OnTreeClick(wxTreeEvent &event);

    private: wxTreeCtrl *treeCtrl;

    private: RenderControl *renderControl;

    private: OgreVisual *visual;
    private: Scene *scene;
    private: Light *dirLight;
  };
}

#endif
