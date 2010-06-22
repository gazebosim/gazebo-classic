#include <stdlib.h>
#include <boost/bind.hpp>
#include <gazebo/gazeboserver.hh>

#define LOG true

namespace gazebo
{

  class PioneerGripper : public Plugin
  {
    public: PioneerGripper() : Plugin()
    {
      //for (double i=0.001; i > 1e-5; i*=0.5) 
        //this->stepTimes.push_back(i);
      this->stepTimes.push_back(0.0001);

      //for (unsigned int i=10; i<=100; i+=10)
        //this->stepIters.push_back(10);
      this->stepIters.push_back(10);

      //this->stepTypes.push_back("quick");
      this->stepTypes.push_back("world");
      //this->stepTypes.push_back("robust");

      this->stepTypesIter = this->stepTypes.begin();
      this->stepTimesIter = this->stepTimes.begin();
      this->stepItersIter = this->stepIters.begin();

      this->path = std::string("/home/nate/work/simpar/data/pioneer_gripper/");
      system( (std::string("mkdir -p ") + this->path).c_str() );

      this->count = 1;

      if (LOG)
      {
        this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "a");
        fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
      }
    }

    public: ~PioneerGripper()
    {
      if (LOG)
        fclose(this->indexFile);

      World::Instance()->DisconnectWorldUpdateStartSignal(
          boost::bind(&PioneerGripper::UpdateCB, this));

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
      this->box->Reset();
    }

    public: void Load()
    {
      std::string model_name = "pioneer_gripper";
      this->robot = (Model*)World::Instance()->GetEntityByName(model_name);
      this->box =  (Model*)World::Instance()->GetEntityByName("custom_box");
      Simulator::Instance()->SetPaused(false);

      if (this->robot)
      {
        this->leftPaddle = (Body*)this->robot->GetChild("left_paddle_body");
        this->rightPaddle = (Body*)this->robot->GetChild("right_paddle_body");
        this->liftBody = (Body*)this->robot->GetChild("lift_body");

        World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&PioneerGripper::UpdateCB, this));

        this->prevTime = Simulator::Instance()->GetSimTime();

        this->physics = World::Instance()->GetPhysicsEngine();

        if (LOG)
        {
          Logger::Instance()->AddLog("pioneer_gripper::left_paddle_body","/tmp/pioneer_left.log");
          Logger::Instance()->AddLog("pioneer_gripper::right_paddle_body","/tmp/pioneer_right.log");
          Logger::Instance()->AddLog("custom_box","/tmp/box.log");
        }

        this->physics->SetStepTime( *this->stepTimesIter );
        this->physics->SetStepType( *this->stepTypesIter );
        this->physics->SetSORPGSIters( *this->stepItersIter );
        std::cout << "Type[" << *this->stepTypesIter << "] " 
          << "Time[" << *this->stepTimesIter << "] "
          << "Iters[" << *this->stepItersIter << "]\n";
      }
      else
        std::cerr << "Unable to find model[" << model_name << "]\n";

      this->spawned = false; this->leftForce = 0;
      this->rightForce = 0;
      this->resetCounter = 10;
    }

    public: void Open()
    {
      Vector3 leftVel = this->leftPaddle->GetWorldLinearVel();
      Vector3 rightVel = this->rightPaddle->GetWorldLinearVel();

      this->leftForce = (0.08 - leftVel.y) * 10.5;
      this->rightForce = (-0.08 - rightVel.y) * 10.5;
    }

    public: void Close()
    {
      Vector3 leftVel = this->leftPaddle->GetWorldLinearVel();
      Vector3 rightVel = this->rightPaddle->GetWorldLinearVel();

      this->leftForce = (-0.08 - leftVel.y) * 0.5;
      this->rightForce = (0.08 - rightVel.y) * 0.5;
    }

    public: void Grip()
    {
      this->leftForce = -500.0;
      this->rightForce = 500.0;
    }

    public: void Lift()
    {
      this->liftBody->SetForce(Vector3(0,0,10.0));
    }

    public: bool IsOpen()
    {
      return this->leftPaddle->GetWorldPose().pos.y >= 0.13 &&
             this->rightPaddle->GetWorldPose().pos.y <= -0.13;
    }

    public: bool IsClosed()
    {
      double diff = this->leftPaddle->GetWorldPose().pos.y - 
                    this->rightPaddle->GetWorldPose().pos.y;

      return fabs(diff) < 0.130;
    }

    public: bool IsUp()
            {
              return this->liftBody->GetWorldPose().pos.z > 0.08;
            }

    public: void UpdateCB()
    {
      if (this->resetCounter > 0)
      {
        this->resetCounter--;
        return;
      }
      else
        Simulator::Instance()->SetPaused(false);

      Time simTime = Simulator::Instance()->GetSimTime();

      this->leftPaddle->SetForce(Vector3(0.0,this->leftForce,0.0));
      this->rightPaddle->SetForce(Vector3(0.0,this->rightForce,0));

      this->Close();

      if (this->IsClosed())
      {
        this->Grip();
        this->Lift();
      }

      //if (!this->IsClosed() || !this->IsUp())
        //this->prevTime = simTime;

      if (simTime - this->prevTime > Time(35,0))
      {
        std::cout << "Done\n";
        //delete this;
        this->UpdateJob();
      }
    }


    public: void UpdateJob()
    {
      if (LOG)
      {
        Logger::Instance()->RemoveLog("pioneer_gripper::left_paddle_body");
        Logger::Instance()->RemoveLog("pioneer_gripper::right_paddle_body");
        Logger::Instance()->RemoveLog("custom_box");

        std::string mv_cmd;
        
        mv_cmd = std::string("mv /tmp/pioneer_left.log ") + this->path + "pioneer_gripper_left_" + boost::lexical_cast<std::string>(this->count) + ".data";
        system(mv_cmd.c_str());

        mv_cmd = std::string("mv /tmp/pioneer_right.log ") + this->path + "pioneer_gripper_right_" + boost::lexical_cast<std::string>(this->count) + ".data";
        system(mv_cmd.c_str());

        mv_cmd = std::string("mv /tmp/box.log ") + this->path + "box_" + boost::lexical_cast<std::string>(this->count) + ".data";
        system(mv_cmd.c_str());

        fprintf(this->indexFile,"%d %s %f %d\n",this->count, 
            (*this->stepTypesIter).c_str(), *this->stepTimesIter, 
            *this->stepItersIter);
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

      /*this->physics->SetStepType( *this->stepTypesIter );
      this->physics->SetStepTime( *this->stepTimesIter );
      this->physics->SetSORPGSIters( *this->stepItersIter );

      this->count++;
      std::cout << "Type[" << *this->stepTypesIter << "] "
                << "Time[" << *this->stepTimesIter << "] " 
                << "Iters[" << *this->stepItersIter << "] "
                << "Count[" << this->count << "]\n";

      //Simulator::Instance()->SetPaused(true);
      this->robot->Reset();
      this->box->Reset();
      this->resetCounter = 10;
      this->prevTime = Simulator::Instance()->GetSimTime();
      */
      delete this;

      /*if (LOG)
        Logger::Instance()->AddLog("pioneer","/tmp/pioneer.log");
        */
    }

    private: std::vector<unsigned int> stepIters;
    private: std::vector<unsigned int>::iterator stepItersIter;

    private: std::vector<double> stepTimes;
    private: std::vector<double>::iterator stepTimesIter;

    private: std::vector<std::string> stepTypes;
    private: std::vector<std::string>::iterator stepTypesIter;
    private: std::string path;
    private: unsigned int count;

    private: Model *robot, *box;
    private: Body *leftPaddle, *rightPaddle, *liftBody;
    private: FILE *indexFile;
    private: Time prevTime;
    private: PhysicsEngine *physics;
    private: int resetCounter;

    private: bool spawned;

    private: double leftForce, rightForce;
  };

  GZ_REGISTER_PLUGIN("PioneerGripper", PioneerGripper)
}
