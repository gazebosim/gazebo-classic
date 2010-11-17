#include <wx/wx.h>

namespace gazebo
{
  class Scene;
  class SimulationFrame;

  class SimulationApp : public wxApp
  {
    /// \brief Constructor
    public: SimulationApp();

    /// \brief Load the simulation app
    public: void Load();

    /// \brief Init the simulation app
    public: void Init();

    public: bool OnInit();

    /// \brief Run the gui
    public: void Run();

    public: void OnIdle(wxTimerEvent &evt);

    /// \brief View a specific scene
    public: void ViewScene( Scene *scene );

    /// \brief Save the gui params in xml format
    public: void Save(std::string &prefix, std::ostream &stream);

    private: SimulationFrame *frame;

    private: wxTimer timer;

  };

  DECLARE_APP(SimulationApp)
}

IMPLEMENT_APP_NO_MAIN(gazebo::SimulationApp)
