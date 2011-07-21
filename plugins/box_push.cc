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
#include "common/Plugin.hh"
#include "common/Console.hh"

#define LOG true

namespace gazebo
{
  namespace common
  {
    class BoxPush : public Plugin
    {
      public: BoxPush() : Plugin() 
      {
        /*this->box = NULL;

        //for (double i=0.001; i > 1e-5; i*=0.5) 
          //this->stepTimes.push_back(i);
        this->stepTimes.push_back(0.001);

        //for (unsigned int i=10; i<=100; i+=10)
        this->stepIters.push_back(100);

        this->stepTypes.push_back("quick");
        this->stepTypes.push_back("world");
        this->stepTypes.push_back("robust");

        this->stepTypesIter = this->stepTypes.begin();
        this->stepTimesIter = this->stepTimes.begin();
        this->stepItersIter = this->stepIters.begin();

        this->path = std::string("/home/nate/work/simpar/data/box_push/");
        system( (std::string("mkdir -p ") + this->path).c_str() );

        if (LOG)
        {
          this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "w");
          fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
        }
        */
      }

      public: ~BoxPush()
      {
        /*if (LOG)
          fclose(this->indexFile);

        World::Instance()->DisconnectWorldUpdateStartSignal(
            boost::bind(&BoxPush::UpdateCB, this));
            */
      }

      public: void Load( sdf::ElementPtr &_sdf )
      {
        std::string modelName = _sdf->GetParent()->GetValueString("name");
        gzdbg << "Load the plugin.Model[" << modelName << "]\n";

        /*this->box = (Model*)World::Instance()->GetEntityByName("box");
        this->physics = World::Instance()->GetPhysicsEngine();
        this->count = 0;
        this->velReached = false;
        this->skipCount = 10;

        World::Instance()->ConnectWorldUpdateStartSignal(
              boost::bind(&BoxPush::UpdateCB, this));

        if (this->box)
        {
          this->box->GetVisualNode()->SetRibbonTrail(true);

          this->prevTime = Simulator::Instance()->GetSimTime();

          if (LOG)
            Logger::Instance()->AddLog("box","/tmp/box.log");

          World::Instance()->ConnectWorldUpdateStartSignal(
              boost::bind(&BoxPush::UpdateCB, this));

          this->physics->SetStepType( *this->stepTypesIter );
          this->physics->SetStepTime( *this->stepTimesIter );
          this->physics->SetSORPGSIters( *this->stepItersIter );

          std::cout << "Type[" << *this->stepTypesIter << "] " 
                    << "Time[" << *this->stepTimesIter << "] "
                    << "Iters[" << *this->stepItersIter << "]\n";
        }
        */
      }

      public: void UpdateCB()
      {
        /*if (this->skipCount > 0)
        {
          this->skipCount--;
          return;
        }

        Time simTime = Simulator::Instance()->GetSimTime();
        Pose3d pose = this->box->GetWorldPose();
        Body *body = (Body*)this->box->GetChild(0);

        Vector3 vel = body->GetWorldLinearVel();

        if (!this->velReached)
          body->SetForce(Vector3(0.2,0,0));
         
        if (!this->velReached && vel.GetLength() > 1.0) 
          this->velReached = true;

        if (this->velReached && vel.GetLength() < 1e-4)
        {
          this->velReached = false;
          this->UpdateJob();
        }
        */
      }

              /*
      public: void UpdateJob()
      {
        if (LOG)
        {
          Logger::Instance()->RemoveLog("box");

          std::string mv_cmd = std::string("mv /tmp/box.log ") + this->path + 
            "box_push_" + boost::lexical_cast<std::string>(this->count) + 
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
              Simulator::Instance()->RemovePlugin(this->handle);
              return;
            }
          }
        }

        this->physics->SetStepType( *this->stepTypesIter );
        this->physics->SetStepTime( *this->stepTimesIter );
        this->physics->SetSORPGSIters( *this->stepItersIter );

        this->skipCount = 10;

        this->count++;
        std::cout << "Type[" << *this->stepTypesIter << "] "
                  << "Time[" << *this->stepTimesIter << "] " 
                  << "Iters[" << *this->stepItersIter << "] "
                  << "Count[" << this->count << "]\n";

        this->box->Reset();
        this->prevTime = Simulator::Instance()->GetSimTime();

        if (LOG)
          Logger::Instance()->AddLog("box","/tmp/box.log");
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
      private: bool velReached;
      private: int skipCount;
    */
      //private: physics::ModelPtr model;
    };

    GZ_REGISTER_PLUGIN("BoxPush", BoxPush)
  }
}
