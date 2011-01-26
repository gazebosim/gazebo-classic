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
#include <boost/bind.hpp>

#include <gazebo/gazeboserver.hh>

#define LOG true

namespace gazebo
{

  class BoxStack : public Plugin
  {
    public: BoxStack() : Plugin() 
    {
      this->box = NULL;

      //for (double i=0.001; i > 1e-5; i*=0.5) 
        //this->stepTimes.push_back(i);
      this->stepTimes.push_back(0.01);

      //for (unsigned int i=10; i<=100; i+=10)
      this->stepIters.push_back(100);

      //this->stepTypes.push_back("quick");
      //this->stepTypes.push_back("world");
      this->stepTypes.push_back("robust");

      this->stepTypesIter = this->stepTypes.begin();
      this->stepTimesIter = this->stepTimes.begin();
      this->stepItersIter = this->stepIters.begin();

      this->path = std::string("/home/nate/work/simpar/data/box_stack/");
      system( (std::string("mkdir -p ") + this->path).c_str() );

      if (LOG)
      {
        this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "a");
        fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
      }
    }

    public: ~BoxStack()
    {
      if (LOG)
        fclose(this->indexFile);

      World::Instance()->DisconnectWorldUpdateStartSignal(
          boost::bind(&BoxStack::UpdateCB, this));
    }

    public: void Load()
    {
      this->box = (Model*)World::Instance()->GetEntityByName("board");
      this->physics = World::Instance()->GetPhysicsEngine();
      this->count = 2;

      World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&BoxStack::UpdateCB, this));

      if (this->box)
      {
        this->prevTime = Simulator::Instance()->GetSimTime();

        if (LOG)
          Logger::Instance()->AddLog("board::body::COM_Entity::geom","/tmp/board.log");

        World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&BoxStack::UpdateCB, this));

        this->physics->SetStepType( *this->stepTypesIter );
        this->physics->SetStepTime( *this->stepTimesIter );
        this->physics->SetSORPGSIters( *this->stepItersIter );

        std::cout << "Type[" << *this->stepTypesIter << "] " 
                  << "Time[" << *this->stepTimesIter << "] "
                  << "Iters[" << *this->stepItersIter << "]\n";
      }
      Simulator::Instance()->SetPaused(false);
    }

    public: void UpdateCB()
    {
      if (Simulator::Instance()->GetSimTime() - this->prevTime > 20.0)
        this->UpdateJob();
    }

    public: void UpdateJob()
    {
      if (LOG)
      {
        Logger::Instance()->RemoveLog("board::body::COM_Entity::geom");

        std::string mv_cmd = std::string("mv /tmp/board.log ") + this->path + 
          "box_stack_" + boost::lexical_cast<std::string>(this->count) + 
          ".data";

        fprintf(this->indexFile,"%d %s %f %d\n",this->count, 
            (*this->stepTypesIter).c_str(), *this->stepTimesIter, 
            *this->stepItersIter);

        system(mv_cmd.c_str());
      }

      this->stepItersIter++;
      if (this->stepItersIter == this->stepIters.end())
      {
        this->stepItersIter = this->stepIters.begin();
        this->stepTimesIter++;
        if (this->stepTimesIter == this->stepTimes.end())
        {
          this->stepTimesIter = this->stepTimes.begin();
          this->stepTypesIter++;
          if (this->stepTypesIter == this->stepTypes.end())
          {
            Simulator::Instance()->SetUserQuit();
            Simulator::Instance()->RemovePlugin(this->handle);
            return;
          }
        }
      }

      this->physics->SetStepType( *this->stepTypesIter );
      this->physics->SetStepTime( *this->stepTimesIter );
      this->physics->SetSORPGSIters( *this->stepItersIter );

      World::Instance()->Reset();

      this->count++;
      std::cout << "Type[" << *this->stepTypesIter << "] "
                << "Time[" << *this->stepTimesIter << "] " 
                << "Iters[" << *this->stepItersIter << "] "
                << "Count[" << this->count << "]\n";

      this->prevTime = Simulator::Instance()->GetSimTime();

      if (LOG)
        Logger::Instance()->AddLog("board::body::COM_Entity::geom","/tmp/board.log");
    }

    private: std::vector<unsigned int> stepIters;
    private: std::vector<unsigned int>::iterator stepItersIter;

    private: std::vector<double> stepTimes;
    private: std::vector<double>::iterator stepTimesIter;

    private: std::vector<std::string> stepTypes;
    private: std::vector<std::string>::iterator stepTypesIter;
    private: std::string path;

    private: Model *box;

    private: FILE *indexFile;
    private: Time prevTime;
    private: PhysicsEngine *physics;
    private: unsigned int count;
  };

  GZ_REGISTER_PLUGIN("BoxStack", BoxStack)
}
