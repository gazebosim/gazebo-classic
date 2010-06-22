#ifndef TOOLBASE_HH
#define TOOLBASE_HH

#include <map>
#include <vector>
#include <yaml.h>

#include "libgazebo/gz.h"

class Toolbase
{
  /// \brief Constructor
  public: Toolbase();

  /// \brief Destructor
  public: virtual ~Toolbase();

  /// \brief Init the tool
  public: bool Init(int argc, char **argv);

  /// \brief Run the tool
  public: virtual bool Run();

  /// \brief Pause the simulation
  public: void Pause();

  /// \brief Run the simulation
  public: void Unpause();

  /// \brief Step the simulation
  public: void Step();

  /// \brief Pause the simulation
  public: void Reset();

  public: virtual void Help() = 0;

  /// \brief Parse yaml parameters
  private: void ParseYAML();

  /// \brief Print all the commands to stdout
  protected: void PrintCommands(std::string prefix);

  protected: libgazebo::Client *client;
  protected: libgazebo::SimulationIface *simIface;
  protected: libgazebo::FactoryIface *factoryIface;

  protected: std::map<std::string, std::string> yamlValues;
  protected: std::vector<std::string> params;

  protected: yaml_parser_t parser;
  protected: yaml_event_t event;


};

#endif
