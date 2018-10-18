#include <wx/wx.h>

namespace gazebo
{
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

    /// \brief Update the gui
    public: void Update();

    /// \brief Run the gui
    public: void Run();

    public: void OnIdle(wxTimerEvent &evt);

    /// \brief Save the gui params in xml format
    public: void Save(std::string &prefix, std::ostream &stream);

    private: SimulationFrame *frame;

    private: wxTimer timer;

  };

  DECLARE_APP(SimulationApp)
}

IMPLEMENT_APP_NO_MAIN(gazebo::SimulationApp)
