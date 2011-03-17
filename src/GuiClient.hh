#ifndef GUICLIENT_HH
#define GUICLIENT_HH

#include "gazebo_config.h"

namespace gazebo
{
  namespace transport
  {
    class Client;
  }

  namespace gui
  {
    class SimulationApp;
  }

  class GuiClient
  {
    public: GuiClient();
    public: virtual ~GuiClient();
    public: void Load(const std::string &filename);

    public: void Run();

    private: bool renderEngineEnabled, guiEnabled;
    private: gui::SimulationApp *gui;
    private: transport::Client *client;
  };
}

#endif
