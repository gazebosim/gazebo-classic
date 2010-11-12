#ifndef SIMULATIONIFACEHANDLER_HH
#define SIMULATIONIFACEHANDLER_HH

namespace libgazebo
{
  class SimulationIface;
  class Common;
}

namespace gazebo
{
  class World;

  class SimulationIfaceHandler
  {
    /// \brief Constructor
    public: SimulationIfaceHandler(World *world);

    /// \brief Destructor
    public: virtual ~SimulationIfaceHandler();

    /// \brief Finialize
    public: void Fini();

    /// \brief Update the sim iface
    public: void Update();

    /// \brief Get the names of interfaces defined in the tree of a model
    private: void GetInterfaceNames(Common *c, std::vector<std::string>& list);

    private: World *world;

    /// Simulation interface
    private: libgazebo::SimulationIface *iface;

  };
}

#endif
