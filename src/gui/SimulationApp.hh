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
#ifndef SIMULATIONAPP_HH
#define SIMULATIONAPP_HH

#include <wx/wx.h>

namespace gazebo
{
  namespace rendering
  {
    class Scene;
  }

	namespace gui
  {
    class SimulationFrame;
  
    class SimulationApp : public wxApp
    {
      /// \brief Constructor
      public: SimulationApp();
 
      /// \brief Destructor 
      public: virtual ~SimulationApp();

      /// \brief Load the simulation app
      public: void Load();
  
      /// \brief Init the simulation app
      public: void Init();
  
      public: bool OnInit();
  
      /// \brief Run the gui
      public: void Run();
  
      public: void OnIdle(wxTimerEvent &evt);
  
      /// \brief View a specific scene
      public: void ViewScene( rendering::Scene *scene );
  
      /// \brief Save the gui params in xml format
      public: void Save(std::string &prefix, std::ostream &stream);
  
      private: SimulationFrame *frame;
  
      private: wxTimer timer;
  
    };

    DECLARE_APP(SimulationApp)
  }
}
IMPLEMENT_APP_NO_MAIN(gazebo::gui::SimulationApp)

#endif
