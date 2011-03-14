/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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
