/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Handles messages from the simulation iface
 * Author: Nate Koenig
 * Date: 8 Nov 2010
 */

#include "Simulator.hh"
#include "Common.hh"
#include "Entity.hh"
#include "Model.hh"
#include "Body.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "Joint.hh"
#include "Logger.hh"
#include "gz.h"

#include "SimulationIfaceHandler.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
///  Constructor
SimulationIfaceHandler::SimulationIfaceHandler()
{
  this->iface = new libgazebo::SimulationIface();
  this->iface->Create( World::Instance()->GetGzServer(), "default" );
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SimulationIfaceHandler::~SimulationIfaceHandler()
{
  delete this->iface;
  this->iface = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize
void SimulationIfaceHandler::Fini()
{
  this->iface->Destroy();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the sim iface
void SimulationIfaceHandler::Update()
{
  libgazebo::SimulationRequestData *response = NULL;

  //TODO: Move this method to simulator? Hard because of the models
  this->iface->Lock(1);

  response = this->iface->data->responses;

  this->iface->data->simTime = Simulator::Instance()->GetSimTime().Double();
  this->iface->data->pauseTime = Simulator::Instance()->GetPauseTime().Double();
  this->iface->data->realTime = Simulator::Instance()->GetRealTime().Double();
  this->iface->data->stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime().Double();
  this->iface->data->state = !Simulator::Instance()->IsPaused();

  unsigned int requestCount = this->iface->data->requestCount;

  // Make sure the request count is valid
  if (this->iface->data->requestCount > GAZEBO_SIMULATION_MAX_REQUESTS)
  {
    gzerr(0) << "Request count[" << this->iface->data->requestCount << "] greater than max allowable[" << GAZEBO_SIMULATION_MAX_REQUESTS << "]\n";

    requestCount = GAZEBO_SIMULATION_MAX_REQUESTS;
  }

  // Process all the requests
  for (unsigned int i=0; i < requestCount; i++)
  {
    libgazebo::SimulationRequestData *req = &(this->iface->data->requests[i]);

    switch (req->type)
    {
      case libgazebo::SimulationRequestData::UNPAUSE: 
        Simulator::Instance()->SetPaused(false);
        break;
      case libgazebo::SimulationRequestData::PAUSE: 
        Simulator::Instance()->SetPaused(
            !Simulator::Instance()->IsPaused());
        break;

      case libgazebo::SimulationRequestData::STEP: 
        Simulator::Instance()->SetStepInc(true);
        break;

      case libgazebo::SimulationRequestData::RESET:
        World::Instance()->Reset();
        break;

      case libgazebo::SimulationRequestData::SAVE:
        Simulator::Instance()->Save();
        break;

      case libgazebo::SimulationRequestData::SET_ENTITY_PARAM_VALUE:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common)
            common->SetParam( req->strValue, req->strValue1 );
          else
            gzerr(0) << "Invalid entity name[" << req->name 
                     << "] in simulation interface Set Param Request.\n";
          break;
        }

      case libgazebo::SimulationRequestData::APPLY_FORCE:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(BODY))
          {
            Body *body = (Body*)(common);

            Vector3 force(req->vec3Value.x, req->vec3Value.y, req->vec3Value.z);

            body->SetForce(force);
          }
          else
          {
            gzerr(0) << "Invalid body name[" << req->name 
              << "] in simulation interface Set Force Request.\n";
          }
          break;

        }

      case libgazebo::SimulationRequestData::APPLY_TORQUE:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(BODY))
          {
            Body *body = (Body*)(common);

            Vector3 torque(req->vec3Value.x,req->vec3Value.y, req->vec3Value.z);

            body->SetTorque(torque);
          }
          else
          {
            gzerr(0) << "Invalid body name[" << req->name 
              << "] in simulation interface Set Torque Request.\n";
          }
          break;

        }
      case libgazebo::SimulationRequestData::SET_LINEAR_VEL:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)(common);

            Vector3 linearVel( req->modelLinearVel.x, req->modelLinearVel.y,
                               req->modelLinearVel.z);

            Pose3d pose = model->GetWorldPose();
            linearVel = pose.rot.RotateVector(linearVel);
            model->SetLinearVel(linearVel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Vel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_ANGULAR_VEL:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && (common->HasType(MODEL) || 
              common->HasType(BODY)))
          {

            Vector3 vel( req->modelAngularVel.x, req->modelAngularVel.y,
                         req->modelAngularVel.z);

            Pose3d pose = ((Entity*)common)->GetWorldPose();
            vel = pose.rot.RotateVector(vel);

            if (common->HasType(MODEL))
              ((Model*)common)->SetAngularVel(vel);
            else if (common->HasType(BODY))
              ((Body*)common)->SetAngularVel(vel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Angular Vel Request.\n";
          }
          break;
        }

 
      case libgazebo::SimulationRequestData::SET_LINEAR_ACCEL:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;

            Vector3 accel( req->modelLinearAccel.x, req->modelLinearAccel.y,
                               req->modelLinearAccel.z);

            Pose3d pose = model->GetWorldPose();
            accel = pose.rot.RotateVector(accel);
            model->SetLinearAccel(accel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Accel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_ANGULAR_ACCEL:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;
            Vector3 accel( req->modelAngularAccel.x, req->modelAngularAccel.y,
                               req->modelAngularAccel.z);

            Pose3d pose = model->GetWorldPose();
            accel = pose.rot.RotateVector(accel);
            model->SetAngularAccel(accel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
              << "] in simulation interface Set Linear Accel Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_STATE:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;

            Pose3d pose;
            Vector3 linearVel( req->modelLinearVel.x,
                               req->modelLinearVel.y,
                               req->modelLinearVel.z);
            Vector3 angularVel( req->modelAngularVel.x,
                                req->modelAngularVel.y,
                                req->modelAngularVel.z);
            Vector3 linearAccel( req->modelLinearAccel.x,
                                 req->modelLinearAccel.y,
                                 req->modelLinearAccel.z);
            Vector3 angularAccel( req->modelAngularAccel.x,
                                  req->modelAngularAccel.y,
                                  req->modelAngularAccel.z);


            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;
            pose.pos.z = req->modelPose.pos.z;

            // The the model's pose
            pose.rot.SetFromEuler(
                Vector3(
                  req->modelPose.roll, 
                  req->modelPose.pitch,
                  req->modelPose.yaw));
            model->SetWorldPose(pose);

            linearVel = pose.rot.RotateVector(linearVel);
            angularVel = pose.rot.RotateVector(angularVel);

            linearAccel = pose.rot.RotateVector(linearAccel);
            angularAccel = pose.rot.RotateVector(angularAccel);

            // Set the model's linear and angular velocity
            model->SetLinearVel(linearVel);
            model->SetAngularVel(angularVel);

            // Set the model's linear and angular acceleration
            model->SetLinearAccel(linearAccel);
            model->SetAngularAccel(angularAccel);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name 
                     << "] in simulation interface Set State Request.\n";
          }
          break;
        }

      case libgazebo::SimulationRequestData::SET_POSE3D:
        {
          Pose3d pose;
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)(common);
            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;
            pose.pos.z = req->modelPose.pos.z;

            pose.rot.SetFromEuler(
                Vector3(req->modelPose.roll, 
                  req->modelPose.pitch,
                  req->modelPose.yaw));
            model->SetWorldPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Set Pose 3d Request.\n";
          }

          break;
        }

      case libgazebo::SimulationRequestData::GET_NUM_MODELS:
        {
          response->type= req->type;
          response->uintValue = World::Instance()->GetModelCount();
          response++;
          this->iface->data->responseCount += 1;
          break;
        }

      case libgazebo::SimulationRequestData::GET_NUM_CHILDREN:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common)
          {
            response->type= req->type;
            response->uintValue = common->GetChildCount();
            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface Get Num Children.\n";
          break;
        }

      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_KEY:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common)
          {
            Param *param = common->GetParam(req->uintValue);
            response->type= req->type;
            memset(response->strValue, 0, 512);

            if (param)
              strncpy(response->strValue, param->GetKey().c_str(), 512);

            response->strValue[511] = '\0';

            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_VALUE:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common)
          {
            Param *param = common->GetParam(req->uintValue);
            response->type= req->type;
            memset(response->strValue, 0, 512);

            if (param)
              strncpy(response->strValue, param->GetAsString().c_str(), 512);

            response->strValue[511] = '\0';
            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM.\n";

          break;
        }



      case libgazebo::SimulationRequestData::GET_ENTITY_PARAM_COUNT:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common)
          {
            unsigned int count = common->GetParamCount();
            response->type= req->type;
            response->uintValue = count;
            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface GET_ENTITY_PARAM_COUNT.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_NAME:
        {
          unsigned int index = req->uintValue;

          if (index < World::Instance()->GetModelCount())
          {
            Model *model = World::Instance()->GetModel(index);
            memset(response->name, 0, 512);

            strncpy(response->name, model->GetCompleteScopedName().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Name.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_CHILD_NAME:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common)
          {
            Common *child;
            unsigned int index;
            response->type= req->type;

            index = req->uintValue;

            child = common->GetChild(index);
            if (child)
            {
              memset(response->strValue, 0, 512);
              strncpy(response->name, child->GetCompleteScopedName().c_str(), 512);
              response->strValue[511] = '\0';

              response++;
              this->iface->data->responseCount += 1;
            }
            else
            gzerr(0) << "Invalid child index in simulation interface Get Child Name.\n";
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Child Name.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_FIDUCIAL_ID:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;
            response->type = req->type;
            response->uintValue = model->GetLaserFiducialId();
            response++;
            this->iface->data->responseCount += 1;
            break;
          } 
        }
      case libgazebo::SimulationRequestData::GET_ENTITY_TYPE:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common)
          {
            response->type = req->type;
            memset(response->strValue, 0, 512);
            if (common->HasType(MODEL))
              strncpy(response->strValue, "model", 512);
            else if (common->HasType(BODY))
              strncpy(response->strValue, "body", 512);
            else if (common->HasType(GEOM))
              strncpy(response->strValue, "geom", 512);

            response->strValue[511] = '\0';
            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid entity name[" << req->name << "] in simulation interface Get Model Type.\n";

          break;
        }
      case libgazebo::SimulationRequestData::GET_MODEL_TYPE:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;

            response->type = req->type;
            memset(response->strValue, 0, 512);
            strncpy(response->strValue, model->GetModelType().c_str(), 512);
            response->strValue[511] = '\0';

            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Type.\n";
          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_EXTENT:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;
            Vector3 min, max;
            model->GetBoundingBox(min, max);

            response->type = req->type;
            strcpy( response->name, req->name);
            response->vec3Value.x = max.x - min.x;
            response->vec3Value.y = max.y - min.y;
            response->vec3Value.z = max.z - min.z;

            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Model Extent.\n";

          break;
        }

      case libgazebo::SimulationRequestData::GET_STATE:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common && common->HasType(ENTITY))
          {
            Entity *ent = (Entity*)(common);
            Pose3d pose;
            Vector3 linearVel;
            Vector3 angularVel;
            Vector3 linearAccel;
            Vector3 angularAccel;

            pose = ent->GetWorldPose();

            // Get the model's linear and angular velocity
            linearVel = ent->GetWorldLinearVel();
            angularVel = ent->GetWorldAngularVel();

            // Get the model's linear and angular acceleration
            linearAccel = ent->GetWorldLinearAccel();
            angularAccel = ent->GetWorldAngularAccel();

            response->modelPose.pos.x = pose.pos.x;
            response->modelPose.pos.y = pose.pos.y;
            response->modelPose.pos.z = pose.pos.z;

            response->modelPose.roll = pose.rot.GetAsEuler().x;
            response->modelPose.pitch = pose.rot.GetAsEuler().y;
            response->modelPose.yaw = pose.rot.GetAsEuler().z;

            response->modelLinearVel.x = linearVel.x;
            response->modelLinearVel.y = linearVel.y;
            response->modelLinearVel.z = linearVel.z;

            response->modelAngularVel.x = angularVel.x;
            response->modelAngularVel.y = angularVel.y;
            response->modelAngularVel.z = angularVel.z;

            response->modelLinearAccel.x = linearAccel.x;
            response->modelLinearAccel.y = linearAccel.y;
            response->modelLinearAccel.z = linearAccel.z;

            response->modelAngularAccel.x = angularAccel.x;
            response->modelAngularAccel.y = angularAccel.y;
            response->modelAngularAccel.z = angularAccel.z;

            response++;
            this->iface->data->responseCount += 1;
          }
          else
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get State Request.\n";
          break;
        }
 
      case libgazebo::SimulationRequestData::GET_POSE2D:
      case libgazebo::SimulationRequestData::GET_POSE3D:
        {
          Common *common = Common::GetByName((char*)req->name);
          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;

            Pose3d pose = model->GetWorldPose();
            Vector3 rot = pose.rot.GetAsEuler();

            response->type = req->type;

            strcpy( response->name, req->name);
            response->modelPose.pos.x = pose.pos.x;
            response->modelPose.pos.y = pose.pos.y;
            response->modelPose.pos.z = pose.pos.z;

            response->modelPose.roll = rot.x;
            response->modelPose.pitch = rot.y;
            response->modelPose.yaw = rot.z;

            response++;
            this->iface->data->responseCount += 1;
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Pose 3d Request.\n";
          }

          break;
        }

      case libgazebo::SimulationRequestData::GET_INTERFACE_TYPE:
        {
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->name, req->name);

          for (unsigned int i = 0; i < World::Instance()->GetModelCount(); i++)
            GetInterfaceNames(World::Instance()->GetModel(i), list);

          std::string mname = req->name;		
          unsigned int i=mname.find(".");        

          while(i!= std::string::npos)
          {
            mname.erase(i,1);
            mname.insert(i,"::");
            i= mname.find(".");
          }

          std::vector<std::string> candids;

          for(unsigned int j=0;j<list.size();j++)
          {
            size_t ind = list[j].find(mname);
            if( ind==0 && ind != std::string::npos && 
               list[j].size() > mname.size())
            {
              candids.push_back(list[j].substr(ind+mname.size(),
                    list[j].size()-ind-mname.size()) );
            }
          }

          for(i=0; i<candids.size(); i++)
          {
            if(candids[i][0]=='>')
            {
              strcpy(response->strValue,
                     candids[i].substr(2,candids[i].size()-2).c_str());
              response->strValue[511]='\0';
              i=candids.size()+5;
            }
          }

          if(strcmp(response->strValue,"irarray")==0)
          {
            strcpy(response->strValue,"ranger");
            response->strValue[511]='\0';		
          }

          if(i<candids.size()+4) // the model is not an interface
          {
            strcpy(response->strValue,"unkown");
            response->strValue[511]='\0';
          }

          response++;
          this->iface->data->responseCount += 1;

          break;
        }

      case libgazebo::SimulationRequestData::GET_MODEL_INTERFACES:
        {
          response->nChildInterfaces=0;
          std::vector<std::string> list;

          response->type = req->type;
          strcpy( response->name, req->name);
          std::vector<Model*>::iterator mmiter;


          for (unsigned int i=0; i < World::Instance()->GetModelCount(); i++)
            this->GetInterfaceNames(World::Instance()->GetModel(i), list);


          // removing the ">>type" from the end of each interface names 
          for(unsigned int jj=0;jj<list.size();jj++){
            unsigned int index = list[jj].find(">>");
            if(index !=std::string::npos)
              list[jj].replace(index,list[jj].size(),"");
          }
	  
          // removing the ">>type" from the end of each interface names 
          for(unsigned int jj=0;jj<list.size();jj++)
          {
            unsigned int index = list[jj].find(">>");
            if(index !=std::string::npos)
              list[jj].replace(index,list[jj].size(),"");
          }

          if(strcmp((char*)req->name,"")==0)
          {
            std::vector<std::string> chlist;
            for(unsigned int i=0;i<list.size();i++)
            {



              std::string str = list[i].substr(0,list[i].find("::"));
              std::vector<std::string>::iterator itr;
              itr = std::find(chlist.begin(),chlist.end(), str);

              if(itr!=chlist.end() || str=="")
                continue;

              unsigned int ii=str.find("::");        
              while(ii!= std::string::npos){

                str.erase(ii,2);
                str.insert(ii,".");
                ii= str.find("::");
              }

              chlist.push_back(str);
              strcpy(response->childInterfaces[response->nChildInterfaces++],str.c_str());
              response->childInterfaces[response->nChildInterfaces-1][511]='\0';

            }

          }
          else
          {
            std::vector<std::string> newlist;
            std::string mname = (char*)req->name;

            size_t i=mname.find(".");        
            while( i != std::string::npos)
            {
              mname.erase(i,1);
              mname.insert(i,"::");
              i= mname.find(".");
            }

            for(unsigned int j=0;j<list.size();j++)
            {
              unsigned int ind = list[j].find(mname);
              if(ind==0 && ind!=std::string::npos && 
                  list[j].size() > mname.size())
              {
                newlist.push_back(list[j].substr(ind+mname.size()+2,
                      list[j].size()-ind-mname.size()-2));
              }
            }

            std::vector<std::string> chlist;
            for( i=0;i<newlist.size();i++)
            {
              unsigned int indx = newlist[i].find("::");
              indx = (indx==std::string::npos)?newlist[i].size():indx;
              std::string str = newlist[i].substr(0,indx);
              std::vector<std::string>::iterator itr;
              itr = std::find(chlist.begin(),chlist.end(), str);


              if(itr!=chlist.end() || str=="")
                continue;

              chlist.push_back(str);
              // Adding the parent name to the child name e.g "parent.child" 
              str=mname+"."+str;

              unsigned int i=str.find("::");        
              while(i!=std::string::npos){
                str.erase(i,2);
                str.insert(i,".");
                i= str.find("::");
              }

              strcpy(response->childInterfaces[response->nChildInterfaces++],
                  str.c_str());
              response->childInterfaces[response->nChildInterfaces-1][511]='\0';
            }
          }

          response++;
          this->iface->data->responseCount += 1;

          break;  
        }

     case libgazebo::SimulationRequestData::START_LOG:
        {
          Logger::Instance()->AddLog(req->name, req->strValue);
          break;
        }

     case libgazebo::SimulationRequestData::STOP_LOG:
        {
          Logger::Instance()->RemoveLog(req->name);
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_TIME:
        {
          World::Instance()->GetPhysicsEngine()->SetStepTime(Time(req->dblValue));
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_ITERS:
        {
          World::Instance()->GetPhysicsEngine()->SetSORPGSIters(req->uintValue);
          break;
        }

     case libgazebo::SimulationRequestData::SET_STEP_TYPE:
        {
          World::Instance()->GetPhysicsEngine()->SetStepType(req->strValue);
          break;
        }

     case libgazebo::SimulationRequestData::GET_STEP_TYPE:
        {
          memset(response->strValue, 0, 512);
          strncpy(response->strValue, World::Instance()->GetPhysicsEngine()->GetStepType().c_str(), 512);
          response->strValue[511] = '\0';
          response++;
          this->iface->data->responseCount += 1;
          break;
        }

     case libgazebo::SimulationRequestData::GET_PLUGIN_COUNT:
        {
          response->type= req->type;
          response->uintValue = Simulator::Instance()->GetPluginCount();
          response++;
          this->iface->data->responseCount += 1;
          break;
        }

     case libgazebo::SimulationRequestData::GET_PLUGIN_NAME:
        {
          memset(response->strValue, 0, 512);
          strncpy(response->strValue, Simulator::Instance()->GetPluginName(req->uintValue).c_str(), 512);
          response->strValue[511] = '\0';
          response++;
          this->iface->data->responseCount += 1;
          break;
        }

 
     case libgazebo::SimulationRequestData::ADD_PLUGIN:
        {
          Simulator::Instance()->AddPlugin(req->strValue, req->name);
          break;
        }

     case libgazebo::SimulationRequestData::REMOVE_PLUGIN:
        {
          Simulator::Instance()->RemovePlugin(req->strValue);
          break;
        }

      case libgazebo::SimulationRequestData::SET_POSE2D:
        {
          Common *common = Common::GetByName((char*)req->name);

          if (common && common->HasType(MODEL))
          {
            Model *model = (Model*)common;

            Pose3d pose = model->GetWorldPose();
            Vector3 rot = pose.rot.GetAsEuler();

            pose.pos.x = req->modelPose.pos.x;
            pose.pos.y = req->modelPose.pos.y;

            pose.rot.SetFromEuler(Vector3(rot.x, rot.y,
                  req->modelPose.yaw));
            model->SetWorldPose(pose);
          }
          else
          {
            gzerr(0) << "Invalid model name[" << req->name << "] in simulation interface Get Children Request.\n";
          }
          break;
        }

      default:
        gzerr(0) << "Unknown simulation iface request[" << req->type << "]\n";
        break;
    }

    this->iface->data->requestCount = 0;
  }

  this->iface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Get all the interface names
void SimulationIfaceHandler::GetInterfaceNames(Common *common, 
                                               std::vector<std::string>& list)
{
  if (!common || !common->HasType(MODEL))
    return;

	Model* m = (Model*)(common);

  m->GetModelInterfaceNames(list);
  for (unsigned int i =0; i < common->GetChildCount(); i++)
		this->GetInterfaceNames(common->GetChild(i), list);
}


