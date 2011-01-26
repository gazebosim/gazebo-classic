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

#define LOG false

namespace gazebo
{

  class PioneerCircle : public Plugin
  {
    public: PioneerCircle() : Plugin() 
    {
      //for (double i=0.001; i > 1e-5; i*=0.5) 
        this->stepTimes.push_back(0.001);

      //for (unsigned int i=10; i<=100; i+=10)
        this->stepIters.push_back(10);

      //this->stepTypes.push_back("world");
      this->stepTypes.push_back("quick");
      //this->stepTypes.push_back("robust");

      this->stepTypesIter = this->stepTypes.begin();
      this->stepTimesIter = this->stepTimes.begin();
      this->stepItersIter = this->stepIters.begin();

      this->path = std::string("/home/nate/work/simpar/data/pioneer_circle/");
      system( (std::string("mkdir -p ") + this->path).c_str() );

      this->circleCount = 0;

      if (LOG)
      {
        this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "a");
        fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
      }
    }

    public: ~PioneerCircle()
    {
      if (LOG)
        fclose(this->indexFile);

      World::Instance()->DisconnectWorldUpdateStartSignal(
          boost::bind(&PioneerCircle::UpdateCB, this));

      for (unsigned int i=0; i < this->robot->GetChildCount(); i++)
      {
        Body *body = dynamic_cast<Body*>(this->robot->GetChild(i));
        if (body)
        {
          body->SetForce(Vector3(0,0,0));
          body->SetTorque(Vector3(0,0,0));
          body->SetLinearVel(Vector3(0,0,0));
          body->SetAngularVel(Vector3(0,0,0));
        }
      }
      this->robot->Reset();
    }

    public: void Load()
    {
      std::string model_name = "pioneer";
      this->robot = (Model*)World::Instance()->GetEntityByName(model_name);

      if (this->robot)
      {
        this->startPose = this->robot->GetWorldPose();
        this->robot->GetVisualNode()->SetRibbonTrail(true);
        this->leftWheel = (Body*)this->robot->GetChild("left_wheel");
        this->rightWheel = (Body*)this->robot->GetChild("right_wheel");

        this->physics = World::Instance()->GetPhysicsEngine();

        this->prevTime = Simulator::Instance()->GetSimTime();
        this->startTime = Simulator::Instance()->GetSimTime();

        this->count = 0;
        this->circleCount = 0;

        if (LOG)
          Logger::Instance()->AddLog("pioneer","/tmp/pioneer.log");

        World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&PioneerCircle::UpdateCB, this));

        this->physics->SetStepType( *this->stepTypesIter );
        this->physics->SetStepTime( *this->stepTimesIter );
        this->physics->SetSORPGSIters( *this->stepItersIter );

        std::cout << "Type[" << *this->stepTypesIter << "] " 
                  << "Time[" << *this->stepTimesIter << "] "
                  << "Iters[" << *this->stepItersIter << "]\n";
      }
      else
        std::cerr << "Unable to find model[" << model_name << "]\n";
    }

    public: void UpdateCB()
    {
      Time simTime = Simulator::Instance()->GetSimTime();
      Pose3d pose = this->robot->GetWorldPose();

      double dist = pose.pos.Distance(this->startPose.pos);

      if (simTime - this->startTime > 10 && 
          this->startPose.pos == Vector3(0,0,0))
        this->startPose = pose;

      if (this->circleCount < 3)
      {
        if (this->leftWheel->GetRelativeAngularVel().z < 2.2) 
          this->leftWheel->SetTorque(Vector3(0.0, 0.0, 0.01));

        if (this->rightWheel->GetRelativeAngularVel().z > 2.1)
          this->rightWheel->SetTorque(Vector3(0.0, 0.0, -0.01));

        if (simTime - this->prevTime > 10.0 && dist < this->prevDist )
        {
          this->startPose = pose;
          std::cout << "CIRCLE\n";
          this->prevTime = simTime;
          this->circleCount++;
        }
      }
      else 
        this->UpdateJob();

      this->prevDist = dist;
    }

    public: void UpdateJob()
    {
      if (LOG)
      {
        printf("HERE. Count[%d]\n", this->count);
        Logger::Instance()->RemoveLog("pioneer");

        std::string mv_cmd = std::string("mv /tmp/pioneer.log ") + this->path + 
          "pioneer_circle_" + boost::lexical_cast<std::string>(this->count) + 
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

      this->count++;
      this->circleCount = 0;
      std::cout << "Type[" << *this->stepTypesIter << "] "
                << "Time[" << *this->stepTimesIter << "] " 
                << "Iters[" << *this->stepItersIter << "] "
                << "Count[" << this->count << "]\n";

      this->robot->Reset();
      this->prevTime = Simulator::Instance()->GetSimTime();

      if (LOG)
        Logger::Instance()->AddLog("pioneer","/tmp/pioneer.log");
    }
    private: std::vector<unsigned int> stepIters;
    private: std::vector<unsigned int>::iterator stepItersIter;

    private: std::vector<double> stepTimes;
    private: std::vector<double>::iterator stepTimesIter;

    private: std::vector<std::string> stepTypes;
    private: std::vector<std::string>::iterator stepTypesIter;
    private: std::string path;

    private: Model *robot;
    private: Body *leftWheel;
    private: Body *rightWheel;

    private: FILE *indexFile;
    private: Time prevTime, startTime;
    private: PhysicsEngine *physics;
    private: unsigned int circleCount;
    private: unsigned int count;

    private: double prevDist;
    private: Pose3d startPose;
  };

  GZ_REGISTER_PLUGIN("PioneerCircle", PioneerCircle)
}
