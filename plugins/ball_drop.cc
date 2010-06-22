#include <boost/bind.hpp>

#include <gazebo/gazeboserver.hh>

#define LOG true

namespace gazebo
{
  class BallDrop : public Plugin
  {
    public: BallDrop() : Plugin() 
    {
      this->sphere = NULL;
      this->model_name = "sphere";
      //this->SpawnBall(0,0,5);

      //for (double i=0.001; i > 1e-5; i*=0.5) 
      this->stepTimes.push_back(0.1);

      //for (unsigned int i=10; i<=100; i+=10)
      this->stepIters.push_back(10);

      //this->stepTypes.push_back("robust");
      this->stepTypes.push_back("world");
      //this->stepTypes.push_back("quick");

      this->stepTypesIter = this->stepTypes.begin();
      this->stepTimesIter = this->stepTimes.begin();
      this->stepItersIter = this->stepIters.begin();

      this->path = std::string("/home/nate/work/simpar/data/ball_drop/");
      system( (std::string("mkdir -p ") + this->path).c_str() );

      if (LOG)
      {
        this->indexFile = fopen(std::string(this->path + "index.txt").c_str(), "w");
        fprintf(this->indexFile, "# index step_type step_time step_iterations\n");
      }
    }

    public: ~BallDrop()
    {
      if (LOG)
        fclose(this->indexFile);

      World::Instance()->DisconnectWorldUpdateStartSignal(
          boost::bind(&BallDrop::UpdateCB, this));

      /*for (unsigned int i=0; i < this->sphere->GetChildCount(); i++)
      {
        Body *body = dynamic_cast<Body*>(this->sphere->GetChild(i));
        if (body)
        {
          body->SetForce(Vector3(0,0,0));
          body->SetTorque(Vector3(0,0,0));
          body->SetLinearVel(Vector3(0,0,0));
          body->SetAngularVel(Vector3(0,0,0));
        }
      }
      this->sphere->Reset();
      */
    }

    public: void SpawnBall(double x, double y, double z=1)
    {
      std::ostringstream model;

      model << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";
    
      model << "<model:physical name='" << this->model_name << "'>";
      model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
      model << "  <rpy>0 0 0</rpy>";
      model << "  <body:sphere name='body'>";
      model << "    <geom:sphere name='geom'>";
      model << "      <size>0.25</size>";
      model << "      <mass>10</mass>";
      model << "      <kp>100000000.0</kp>";
      model << "      <kd>1.0</kd>";
      model << "      <bounce>0</bounce>";
      model << "      <bounceVel>100000</bounceVel>";
      model << "      <slip1>0.01</slip1>";
      model << "      <slip2>0.01</slip2>";
      model << "      <enableContacts>true</enableContacts>";
      model << "      <visual>";
      model << "        <size>0.5 0.5 0.5</size>";
      model << "        <mesh>unit_sphere</mesh>";
      model << "        <material>Gazebo/Rocky</material>";
      model << "      </visual>";
      model << "    </geom:sphere>";
      model << "  </body:sphere>";
      model << "</model:physical>";

      model << "</gazebo:world>";

      World::Instance()->InsertEntity( model.str() );
    }

    public: void Load()
    {
      this->physics = World::Instance()->GetPhysicsEngine();
      this->count = 0;
      this->prev_z = 0;

      this->physics->SetStepType( *this->stepTypesIter );
      this->physics->SetStepTime( *this->stepTimesIter );
      this->physics->SetStepTime( *this->stepTimesIter );
      this->physics->SetSORPGSIters( *this->stepItersIter );

      World::Instance()->ConnectWorldUpdateStartSignal(
            boost::bind(&BallDrop::UpdateCB, this));
    }

    public: void UpdateCB()
    {
      if (this->sphere == NULL)
      {
        this->sphere = (Model*)World::Instance()->GetEntityByName(this->model_name);
        if (this->sphere)
        {
          this->sphere->GetVisualNode()->SetRibbonTrail(true);

          this->prevTime = Simulator::Instance()->GetRealTime();

          if (LOG)
            Logger::Instance()->AddLog("sphere::body::COM_Entity::geom","/tmp/sphere.log");

          World::Instance()->ConnectWorldUpdateStartSignal(
              boost::bind(&BallDrop::UpdateCB, this));

          std::cout << "Type[" << *this->stepTypesIter << "] " 
                    << "Time[" << *this->stepTimesIter << "] "
                    << "Iters[" << *this->stepItersIter << "]\n";
        }
      }
      else
      {
        Time realTime = Simulator::Instance()->GetRealTime();
        Pose3d pose = this->sphere->GetWorldPose();

        if ( fabs(prev_z - pose.pos.z) >= 1e-5 )
          this->prevTime = realTime;

        this->prev_z = pose.pos.z;

        if (realTime - this->prevTime > 10.0)
        {
          this->UpdateJob();
        }
      }
    }

    public: void UpdateJob()
    {
      if (LOG)
      {
       // Logger::Instance()->RemoveLog("sphere");

        std::string mv_cmd = std::string("mv /tmp/sphere.log ") + this->path + 
          "ball_drop_" + boost::lexical_cast<std::string>(this->count) + 
          ".data";

        fprintf(this->indexFile,"%d %s %f %d\n",this->count, 
            (*this->stepTypesIter).c_str(), *this->stepTimesIter, 
            *this->stepItersIter);

        //system(mv_cmd.c_str());
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

      //this->sphere->Reset();
      this->prevTime = Simulator::Instance()->GetRealTime();

      /*if (LOG)
        Logger::Instance()->AddLog("sphere","/tmp/sphere.log");
        */
    }

    private: std::vector<unsigned int> stepIters;
    private: std::vector<unsigned int>::iterator stepItersIter;

    private: std::vector<double> stepTimes;
    private: std::vector<double>::iterator stepTimesIter;

    private: std::vector<std::string> stepTypes;
    private: std::vector<std::string>::iterator stepTypesIter;
    private: std::string path;

    private: Model *sphere;

    private: FILE *indexFile;
    private: Time prevTime;
    private: PhysicsEngine *physics;
    private: unsigned int count;

    private: double prev_z;

    private: std::string model_name;
  };

  GZ_REGISTER_PLUGIN("BallDrop", BallDrop)
}
