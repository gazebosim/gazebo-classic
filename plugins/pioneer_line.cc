#include <stdlib.h>
#include <boost/bind.hpp>
#include <gazebo/gazeboserver.hh>

#define LOG false

namespace gazebo
{

  class PioneerLine : public Plugin
  {
    public: PioneerLine() : Plugin()
    {
      //for (double i=0.001; i > 1e-5; i*=0.5) 
      this->stepTimes.push_back(0.001);

      for (unsigned int i=10; i<=100; i+=10)
        this->stepIters.push_back(i);

      this->stepTypes.push_back("quick");
      this->stepTypes.push_back("world");
      this->stepTypes.push_back("robust");

      this->stepTypesIter = this->stepTypes.begin();
      this->stepTimesIter = this->stepTimes.begin();
      this->stepItersIter = this->stepIters.begin();

      this->path = std::string("/home/nate/work/simpar/data/pioneer_line/");
      system( (std::string("mkdir -p ") + this->path).c_str() );

      this->count = 0;

      if (LOG)
      {
        this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "w");
        fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
      }
    }

    public: ~PioneerLine()
    {
      if (LOG)
        fclose(this->indexFile);

      World::Instance()->DisconnectWorldUpdateStartSignal(
          boost::bind(&PioneerLine::UpdateCB, this));

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

        this->robot->GetVisualNode()->SetRibbonTrail(true);
        this->leftWheel = (Body*)this->robot->GetChild("left_wheel");
        this->rightWheel = (Body*)this->robot->GetChild("right_wheel");

        World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&PioneerLine::UpdateCB, this));

        this->prevTime = Simulator::Instance()->GetSimTime();

        this->physics = World::Instance()->GetPhysicsEngine();

        if (LOG)
          Logger::Instance()->AddLog("pioneer","/tmp/pioneer.log");

        this->physics->SetStepTime( *this->stepTimesIter );
        this->physics->SetStepType( *this->stepTypesIter );
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

      if (simTime - prevTime < Time(0,100000000)) 
      {
        this->leftWheel->SetTorque(Vector3(0.0,0.0,0.1));
        this->rightWheel->SetTorque(Vector3(0.0,0.0,0.1));
      }
      else if (simTime - prevTime >= Time(20,0)) 
        this->UpdateJob();
    }


    public: void UpdateJob()
    {
      if (LOG)
      {
        Logger::Instance()->RemoveLog("pioneer");

        std::string mv_cmd = std::string("mv /tmp/pioneer.log ") + this->path + 
          "pioneer_line_" + boost::lexical_cast<std::string>(this->count) + 
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
    private: unsigned int count;

    private: Model *robot;
    private: Body *rightWheel, *leftWheel;
    private: FILE *indexFile;
    private: Time prevTime;
    private: PhysicsEngine *physics;

  };

  GZ_REGISTER_PLUGIN("PioneerLine", PioneerLine)
}
