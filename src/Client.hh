#ifndef CLIENT_HH
#define CLIENT_HH

#include "gazebo_config.h"

namespace gazebo
{
  namespace gui
  {
    class SimulationApp;
  }

  class Client
  {
    public: Client();
    public: virtual ~Client();
    public: void Load(const std::string &filename);

    public: void Run();

    private: bool renderEngineEnabled, guiEnabled;
    private: gui::SimulationApp *gui;
  };
}

#endif
