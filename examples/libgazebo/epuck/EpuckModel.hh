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
