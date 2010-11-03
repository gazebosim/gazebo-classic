#include <wx/wx.h>
#include <wx/treectrl.h>

class wxAuiManager;
class wxAuiManagerEvent;
class wxPropertyGrid;
class wxPropertyGridEvent;

namespace gazebo
{
  class RenderPanel;
  class TimePanel;
  class Entity;
  class Common;

  class SimulationFrame : public wxFrame
  {
    enum ToolbarButtons {PLAY, PAUSE, STEP, BOX, SPHERE, CYLINDER, DIRECTIONAL, POINT, SPOT, CURSOR};
    enum MenuIds {ID_OPEN, ID_LOAD_MESH, ID_SAVE, ID_RESET, ID_WIREFRAME, ID_PHYSICS, ID_BOUNDING, ID_JOINTS, ID_CONTACTS, ID_LIGHTS, ID_CAMERAS, ID_SNAPTOGRID, ID_EDITGRID};

    public: SimulationFrame(wxWindow *parent);

    public: virtual ~SimulationFrame();

    /// \brief Create the cameras
    public: void CreateCameras();

    public: void Init();

    public: void Update();

    private: void OnPause(bool pause);

    private: void OnQuit(wxCommandEvent &event);
    private: void OnOpen(wxCommandEvent &event);
    private: void OnSave(wxCommandEvent &event);
    private: void OnReset(wxCommandEvent &event);
    private: void OnLoadMesh(wxCommandEvent & WXUNUSED(event));

    private: void OnSnapToGrid(wxCommandEvent &event);
    private: void OnEditGrid(wxCommandEvent &event);

    private: void OnWireframe(wxCommandEvent &event);
    private: void OnShowPhysics(wxCommandEvent &event);
    private: void OnShowBoundingBoxes(wxCommandEvent &event);
    private: void OnShowJoints(wxCommandEvent &event);
    private: void OnShowContacts(wxCommandEvent &event);
    private: void OnShowLights(wxCommandEvent &event);
    private: void OnShowCameras(wxCommandEvent &event);

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

    private: void AddEntityHelper(const Common *entity, wxTreeItemId parent);

    private: RenderPanel *renderPanel;
    private: TimePanel *timePanel;

    private: wxAuiManager *auiManager;
    private: wxToolBar *toolbar;

    private: wxTreeCtrl *treeCtrl;
    private: wxPropertyGrid *propGrid;
  };
}
