#ifndef GUICLIENT_HH
#define GUICLIENT_HH

#include <boost/shared_ptr.hpp>

#include "common/Node.hh"
#include "common/Messages.hh"
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

    public: void Quit();

    private: void Test(const boost::shared_ptr<msgs::String const> &msg );

    private: common::NodePtr node;
    private: bool renderEngineEnabled, guiEnabled;
    private: gui::SimulationApp *gui;
    private: transport::Client *client;

    private: bool quit;
  };
}

#endif
