#ifndef EPUCKMODEL_HH_
#define EPUCKMODEL_HH_

#include <sstream>
#include <gazebo/gazebo.h>
using namespace gazebo;

#define	NUMS_IR 8
#define WHEEL_DIAMETER 0.042
#define WHEEL_SEP   0.052


class Epuck
{
	public: 
	Epuck();
    ~Epuck();
	public:const char * CreateEpuckModel(int id, Pose& pose,  bool withCamera=false);
	public:bool Subscribe(Client *client);
	public:void EnableMotor();
	
	public: void DoCircle(double speed, double r, bool anticlock=true);
	
	public:bool SetSpeed(const double left, const double right);
	public:bool GetIRReading( );
	public:void Update();
	
	public:double ir[NUMS_IR];
	
	private: int id;
	private: char  robotName[64];
	private: char   posIfaceName[64];
	private: char   irIfaceName[64];
	private: double lastupdateTime;
    private: double currentTime;
    
	private: PositionIface *posIface;
	private: IRIface *irIface;
	private: int irCount;
	
};


#endif /*EPUCKMODEL_HH_*/
