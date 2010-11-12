#ifndef MODELBUILDER_HH
#define MODELBUILDER_HH

#include <wx/wx.h>

class wxAuiManager;

namespace gazebo
{
  class RenderControl;
  class Scene;
  class Light;
  class Model;

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
    private: Scene *scene;
    private: Light *dirLight;
    private: Model *model;

  };
}

#endif
