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
#include <string.h>
#include <string>
#include <iostream>
#include "EpuckModel.hh"

double minfrontdistance = 0.095;
double minsidedistance = 0.04;
double forwardspeed=0.08;
double maxirrange=0.1;


double weightleft[8]={-1.6, -0.65, -0.3, 0, 0, 0.3, .6, 1.65};
double weightright[8]={1.65, 0.8, 0.3, 0, 0, -0.3, -0.8, -1.6};

Epuck::Epuck()
{
	this->posIface = new  PositionIface();
    this->irIface = new  IRIface();
	this->id = 9999;
	strcpy(this->robotName,"");
	this->lastupdateTime = 0;
	this->currentTime=0;	
	this->irCount = NUMS_IR;
}

Epuck::~Epuck()
{
	//if(this->posIface)
	  delete this->posIface;
	//if(this->irIface)
	  delete this->irIface;
	
}

bool Epuck::SetSpeed(double lspeed, double rspeed)
{
	posIface->Lock(1);
    posIface->data->cmdVelocity.pos.x = (lspeed + rspeed )/2.0;
    posIface->data->cmdVelocity.pos.y = 0;
    posIface->data->cmdVelocity.yaw = (lspeed-rspeed)/( WHEEL_SEP);
    posIface->Unlock();
    return true;
}

bool Epuck::GetIRReading()
{
	
	bool ret=false;
	irIface->Lock(1);
	if(irIface->data->ir_count == NUMS_IR)
	{
      for(int i=0;i<NUMS_IR;i++)
	  {
		ir[i] = irIface->data->ranges[i];
	  }
	  
	  ret = true;
	}
	irIface->Unlock();
	return ret;
}

void Epuck::DoCircle(double speed, double r, bool anticlock)
{
	double w = speed/r;
	double lspeed = w * (r-WHEEL_SEP/2.0 );
	double rspeed = w * (r+WHEEL_SEP/2.0);
	if(anticlock)
	  SetSpeed(lspeed, rspeed);
	else
	  SetSpeed(rspeed, lspeed);
}


bool Epuck::Subscribe(Client * client)
{
  try
  {
    posIface->Open(client, posIfaceName);
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }
  
  try
  {
    irIface->Open(client, irIfaceName);
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the ir interface\n"
    << e << "\n";
    return -1;
  }

  
  return true;
  
	 
}

void Epuck::EnableMotor()
{
  posIface->Lock(1);
  posIface->data->cmdEnableMotors = 1;
  posIface->Unlock();
}


//random walk and avoidance behaviour
void Epuck::Update()
{
	//printf("robot%d update\n",this->id);
	if(GetIRReading()==false)
	{
	  SetSpeed(0,0);
	  return;
	}
	
	double leftspeed=forwardspeed;
    	double rightspeed=forwardspeed;
	
	bool obs = ((ir[0] < minfrontdistance) ||   
               (ir[1] < minfrontdistance) ||
               (ir[6] < minfrontdistance) ||   
               (ir[7] < minfrontdistance) ||
               (ir[2] < minsidedistance)||
               (ir[5] < minsidedistance));
      if(obs )
      {
      	leftspeed = 0.01;
      	rightspeed = 0.01;
      	for(int i=0;i<8;i++)
      	{
      		leftspeed += weightleft[i] * (ir[i]-minfrontdistance);
      		rightspeed += weightright[i] * (ir[i]-minfrontdistance);
      	}
      }
      SetSpeed(leftspeed, rightspeed);
	
	
}


const char * Epuck::CreateEpuckModel( int id, Pose& pose, bool withCamera)
{
	std::ostringstream stream;
    stream << "<model:physical name='robot"<<id<<"'>";
    stream <<      "<xyz>"<<pose.pos.x<<" "<<pose.pos.y<<" "<<pose.pos.z<<"</xyz>";
    stream << 	   "<rpy>"<<pose.roll<<" "<<pose.pitch<<" "<<pose.yaw<<"</rpy>";
    stream << 	   "<static>false</static>";
    stream << 	   "<controller:differential_position2d name='controller"<<id<<"'>";
    stream << 			"<leftJoint>left_wheel_hinge</leftJoint>";
    stream << 			"<rightJoint>right_wheel_hinge</rightJoint>";
    stream << 			"<wheelSeparation>0.052</wheelSeparation>";
    stream << 			"<wheelDiameter>0.042</wheelDiameter>";
    stream << 			"<torque>10</torque>";
    stream << 			"<interface:position name='position_iface_"<<"0"<<"'/>";
    stream << 		"</controller:differential_position2d>";
    
    stream << 	"<canonicalBody>epuck_chassis</canonicalBody>";
		
    stream << 	"<body:box name='epuck_chassis'>";
    stream << 		"<xyz>0.0 0.0 0.02</xyz>";
    stream << 		"<geom:box name='chassis_geom'>";
    stream << 			"<size>0.07 0.04 0.038</size>";
    stream << 			"<mass>0.1</mass>";
    stream << 			"<mul>1</mul>";
    stream << 			"<visual>";
    stream << 				"<rpy>90.0 0.0 90.0</rpy>";
    stream << 				"<mesh>epuck/chassis.mesh</mesh>";
    stream << 				"<material>Gazebo/EpuckBody</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.0 0.0 0.019</xyz>";
    stream << 				"<rpy>0 0 -90</rpy>";
    stream << 				"<size>0.07 0.07 0</size>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<material>Gazebo/EpuckPlate</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>-0.004 0.000 0.0266</xyz>";
    stream << 				"<rpy>90 0 -90</rpy>";
    stream << 				"<mesh>epuck/turret.mesh</mesh>";
    stream << 				"<scale>0.0128 0.0128 0.0128</scale>";
    stream << 				"<material>Gazebo/PCBGreen</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>-0.004 0.000 0.028</xyz>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<rpy>0 0 90</rpy>";
    stream << 				"<size>0.0625 0.0625 0</size>";
    stream << 				"<material>Gazebo/Turret</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.0 0.0 0.017</xyz>";
    stream << 				"<rpy>90.0 0.0 90.0</rpy>";
    stream << 				"<size>0.0725 0.007 0.0725</size>";
    stream << 				"<mesh>epuck/ring.mesh</mesh>";
    stream << 				"<material>Gazebo/EpuckRing</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.0254 0.0193 0.0226</xyz>";
    stream << 				"<rpy> 0.0 0.0 0.0</rpy>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.00225 0.00225 0.01</size>";
    stream << 				"<material>Gazebo/White</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.0254 -0.0193 0.0226</xyz>";
    stream << 				"<rpy> 0.0 0.0 0.0</rpy>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.00225 0.00225 0.01</size>";
    stream << 				"<material>Gazebo/White</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>-0.032 0 0.0226</xyz>";
    stream << 				"<rpy> 0.0 0.0 0.0</rpy>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.00225 0.00225 0.01</size>";
    stream << 				"<material>Gazebo/White</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.03 0.01 0.013</xyz>";
    stream << 				"<rpy>0.0 0.0 17.20</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0.022 0.025 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 45.8</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0 0.031 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 90.0</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>-0.03 0.015 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 151.5</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz> -0.03 -0.015 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 -151.5</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz> 0 -0.031 0.013</xyz>";
    stream << 				"<rpy>0.0 0.0 -90.0</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz> 0.022 -0.025 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 -45.8</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz> 0.03 -0.01 0.013 </xyz>";
    stream << 				"<rpy>0.0 0.0 -17.2</rpy>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<size>0.002 0.007 0.004</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<rpy>0.0 0.0 90.0</rpy>";
    stream << 				"<xyz>0.026 0.0 0.010</xyz>";
    stream << 				"<mesh>epuck/camera.mesh</mesh>";
    stream << 				"<scale>1 1 1</scale>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 		"</geom:box>";
		
    stream << 		"<sensor:ir name='ir_ring"<<id<<"'>";
    stream << 			"<irCount> 8 </irCount>";
    stream << 			"<displayRays>false</displayRays>";
    stream << 			"<ir name='IR_0'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>0.03 0.01 0.013</origin>";
				
    stream << 				"<minAngle>10.2</minAngle>";
    stream << 				"<maxAngle>24.2</maxAngle>";
				
    stream << 				"<minRange>0.01</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_1'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>0.022 0.025 0.013</origin>";
				
    stream << 				"<minAngle>38.8</minAngle>";
    stream << 				"<maxAngle>52.8</maxAngle>";
				
    stream << 				"<minRange>0.005</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_2'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>0 0.031 0.013 </origin>";
				
    stream << 				"<minAngle>83.0</minAngle>";
    stream << 				"<maxAngle>97.0</maxAngle>";
				
    stream << 				"<minRange>0.005</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_3'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>-0.03 0.015 0.013</origin>";
				
    stream << 				"<minAngle>144.5</minAngle>";
    stream << 				"<maxAngle>158.5</maxAngle>";
				
    stream << 				"<minRange>0.01</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_4'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>-0.03 -0.015 0.013</origin>";
				
    stream << 				"<minAngle>-158.5</minAngle>";
    stream << 				"<maxAngle>-144.5</maxAngle>";
				
    stream << 				"<minRange>0.01</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_5'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>0 -0.031 0.013</origin>";
				
    stream << 				"<minAngle>-97.0</minAngle>";
    stream << 				"<maxAngle>-83.0</maxAngle>";
				
    stream << 				"<minRange>0.005</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_6'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin>0.022 -0.025 0.013</origin>";
				
    stream << 				"<minAngle>-52.8</minAngle>";
    stream << 				"<maxAngle>-38.8</maxAngle>";
				
    stream << 				"<minRange>0.005</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<ir name='IR_7'>";
    stream << 				"<rayCount>4</rayCount>";
    stream << 				"<rangeCount>10</rangeCount>";
    stream << 				"<origin> 0.03 -0.01 0.013</origin>";
				
    stream << 				"<minAngle>-24.2</minAngle>";
    stream << 				"<maxAngle>-10.2</maxAngle>";
				
    stream << 				"<minRange>0.01</minRange>";
    stream << 				"<maxRange>0.1</maxRange>";
    stream << 			"</ir>";
    stream << 			"<controller:irarray name='irarray"<<id<<"'>";
    stream << 				"<interface:irarray name='irarray_iface_"<<"0"<<"'/>";
    stream << 			"</controller:irarray>";
    stream << 		"</sensor:ir>";
    stream << 	"</body:box>";
    

	
    stream << 	"<body:cylinder name='left_wheel'>";
    stream << 		"<xyz>0 0.026 0.02</xyz>";
    stream << 		"<rpy>0 90 -90</rpy>";
		
    stream << 		"<geom:cylinder name='left_wheel_geom'>";
    stream << 			"<size>0.03 0.003</size>";
    stream << 			"<mass>0.01</mass>";
    stream << 			"<mul>0.5</mul>";
    stream <<			"<visual>";
    stream << 				"<size>0.04 0.04 0.003</size>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<xyz>0. 0 -0.0016</xyz>";
    stream << 				"<size>0.03 0.03 0</size>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<material>Gazebo/EpuckLogo</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.042 0.042 0.0015</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.012 0.012 0.011</size>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.01 0.01 0.014</size>";
    stream << 				"<material>Gazebo/EpuckGold</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.004 0.004 0.0195</size>";
    stream << 				"<material>Gazebo/EpuckGold</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.008 0.008 0.018</size>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream << 		"</geom:cylinder>";
    stream << 	"</body:cylinder>";
    
    if(withCamera)
    {
    stream <<		"<model:physical name='cam"<<id<<"'>";
    stream << 			"<body:empty name='epuck_camera'>";
    stream << 				"<rpy>0.0 0.0 0.0</rpy>";
    stream << 				"<xyz>0.026 0.0 0.030</xyz>";
    stream << 				"<sensor:camera name='epuck_cam_sensor"<<id<<"'>";
    stream << 					"<imageSize>200 150</imageSize>";
    stream << 					"<hfov>60</hfov>";
    stream << 					"<nearClip>0.02</nearClip>";
    stream << 					"<farClip>20</farClip>";
    stream << 					"<saveFrames>false</saveFrames>";
    stream << 					"<saveFramePath>frames</saveFramePath>";
    stream << 					"<controller:generic_camera name='epuck_camera_controller'>";
    stream <<    				"<interface:camera name='camera_iface_0'/>";
    stream <<  					"</controller:generic_camera>";
    stream << 				"</sensor:camera>";
    stream << 			"</body:empty>";
    stream << 			"<attach>";
    stream << 				"<parentBody>epuck_chassis</parentBody>";
    stream << 				"<myBody>epuck_camera</myBody>";
    stream << 			"</attach>";
    stream <<		"</model:physical>";
    }

	
    stream << 	"<body:cylinder name='right_wheel'>";
    stream << 		"<xyz>0 -0.026 0.02</xyz>";
    stream << 		"<rpy>0 90 90</rpy>";
		
    stream << 		"<geom:cylinder name='right_wheel_geom'>";
    stream << 			"<size>0.03 0.003</size>";
    stream << 			"<mass>0.01</mass>";
    stream << 			"<mul>0.5</mul>";
			
    stream << 			"<visual>";
    stream << 				"<size>0.04 0.04 0.003</size>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream <<           "<visual>";
    stream << 				"<xyz>0. 0 -0.0016</xyz>";
    stream << 				"<size>0.03 0.03 0</size>";
    stream << 				"<mesh>unit_box</mesh>";
    stream << 				"<material>Gazebo/EpuckLogo</material>";
    stream << 			"</visual>";

    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.042 0.042 0.0015</size>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.012 0.012 0.011</size>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.01 0.01 0.014</size>";
    stream << 				"<material>Gazebo/Gold</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.004 0.004 0.0195</size>";
    stream << 				"<material>Gazebo/Gold</material>";
    stream << 			"</visual>";
    stream << 			"<visual>";
    stream << 				"<mesh>unit_cylinder</mesh>";
    stream << 				"<size>0.008 0.008 0.018</size>";
    stream << 				"<material>Gazebo/Grey</material>";
    stream << 			"</visual>";
    stream << 		"</geom:cylinder>";
    stream << 	"</body:cylinder>";
	
    stream << 	"<body:sphere name='castor_body_rear'>";
    stream << 		"<xyz>-0.026 0 -0.000</xyz>";
    stream << 		"<rpy>0 0 0</rpy>";
    stream << 		"<geom:sphere name='castor_geom_rear'>";
    stream << 			"<size>0.01</size>";
    stream << 			"<mass>0.1</mass>";
    stream << 			"<mu1>0.5</mu1>";
			
    stream << 			"<visual>";
    stream << 				"<scale>0.01 0.01 0.01</scale>";
    stream << 				"<mesh>unit_sphere</mesh>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 		"</geom:sphere>";
    stream << 	"</body:sphere>";
	
    stream << 	"<body:sphere name='castor_body_front'>";
    stream << 		"<xyz>0.026 0 -0.000</xyz>";
    stream << 		"<rpy>0 0 0</rpy>";
    stream << 		"<geom:sphere name='castor_geom_front'>";
    stream << 			"<size>0.01</size>";
    stream << 			"<mass>0.1</mass>";
    stream << 			"<mu1>0.5</mu1>";
			
    stream << 			"<visual>";
    stream << 				"<scale>0.01 0.01 0.01</scale>";
    stream << 				"<mesh>unit_sphere</mesh>";
    stream << 				"<material>Gazebo/Black</material>";
    stream << 			"</visual>";
    stream << 		"</geom:sphere>";
    stream << 	"</body:sphere>";
	
    stream << 	"<joint:hinge name='left_wheel_hinge'>";
    stream << 		"<body1>left_wheel</body1>";
    stream << 		"<body2>epuck_chassis</body2>";
    stream << 		"<anchor>left_wheel</anchor>";
    stream << 		"<axis>0 1 0</axis>";
    stream << 		"<erp>0.8</erp>";
    stream << 		"<cfm>10e-5</cfm>";
    stream << 	"</joint:hinge>";
    stream << 	"<joint:hinge name='right_wheel_hinge'>";
    stream << 		"<body1>right_wheel</body1>";
    stream << 		"<body2>epuck_chassis</body2>";
    stream << 		"<anchor>right_wheel</anchor>";
    stream << 		"<axis>0 1 0</axis>";
    stream << 		"<erp>0.8</erp>";
    stream << 		"<cfm>10e-5</cfm>";
    stream << 	"</joint:hinge>";
	
    stream << 	"<joint:ball name='ball_joint_front'>";
    stream << 		"<body1>castor_body_front</body1>";
    stream << 		"<body2>epuck_chassis</body2>";
    stream << 		"<anchor>castor_body_front</anchor>";
    stream << 		"<erp>0.8</erp>";
    stream << 		"<cfm>10e-5</cfm>";
    stream << 	"</joint:ball>";
	
    stream << 	"<joint:ball name='ball_joint_rear'>";
    stream << 		"<body1>castor_body_rear</body1>";
    stream << 		"<body2>epuck_chassis</body2>";
    stream << 		"<anchor>castor_body_rear</anchor>";
    stream << 		"<erp>0.8</erp>";
    stream << 		"<cfm>10e-5</cfm>";
    stream << 	"</joint:ball>";
	
    stream << "</model:physical>";
    
    this->id = id;
    sprintf(this->robotName,"robot%d",id);
    
    sprintf(this->posIfaceName,"robot%d-position_iface_0", id);
    sprintf(this->irIfaceName,"robot%d-irarray_iface_0",id);
    
    
    
    return stream.str().c_str();
    
	
}
