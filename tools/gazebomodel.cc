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
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "Toolbase.hh"

class ModelTool : public Toolbase
{

  //////////////////////////////////////////////////////////////////////////////
  // Print out info for one model. Recurses through all child models
  public: void PrintModel(std::string name, std::string prefix)
  {
    std::string type;
    libgazebo::Pose pose;

    std::cout << prefix << name << "\n";

    unsigned int children;
    if (!this->simIface->GetNumChildren(name, children))
      std::cerr << "Unable to get the number of children for model[" 
        << name << "]\n";

    for (unsigned int i=0; i < children; i++)
    {
      std::string childName;
      if (!this->simIface->GetChildName(name, i, childName))
        std::cerr << "Unable to get model[" << name << "] child name.\n";
      else
      {
        std::string type;
        this->simIface->GetEntityType(childName, type);
        if (type == "model")
          this->PrintModel(childName, prefix+"  ");
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Print out a list of all the models
  public: void List()
  {
    unsigned int numModels = 0;

    std::string prefix = "";
    if (!this->simIface->GetNumModels(numModels))
      std::cerr << "Unable to get the model count\n";

    for (unsigned int i=0; i < numModels; i++)
    {
      std::string name;
      if (!this->simIface->GetModelName(i, name))
        std::cerr << "Unable to get model[" << i << "]\n";

      this->PrintModel(name, "");
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Show info for a model
  public: void Info()
  {
    if (this->params.size() < 2)
      std::cerr << "Missing model name\n";
    else
    {
      for (unsigned int i=1; i < this->params.size(); i++)
      {
        std::string name = params[i];
        std::string type;
        libgazebo::Pose pose;
        libgazebo::Vec3 linearVel, linearAccel, angularVel, angularAccel;
        unsigned int paramCount;

        if (!this->simIface->GetEntityParamCount(name, paramCount))
          std::cerr << "Unable to get model[" << name << "] param count\n";

        this->simIface->GetState(name, pose, linearVel, angularVel, 
                                 linearAccel, angularAccel);

        std::cout << "linear_vel: " << linearVel.x << " " 
                  << linearVel.y << " " << linearVel.z << "\n";
        std::cout << "linear_accel: " << linearAccel.x << " " 
                  << linearAccel.y << " " << linearAccel.z << "\n";

        std::cout << "angular_vel: " << angularVel.x << " " 
                  << angularVel.y << " " << angularVel.z << "\n";
        std::cout << "angular_accel: " << angularAccel.x << " " 
                  << angularAccel.y << " " << angularAccel.z << "\n";
        
        for (unsigned int i=0; i < paramCount; i++)
        {
          std::string paramKey;
          std::string paramValue;

          if (!this->simIface->GetEntityParamKey(name, i, paramKey))
            std::cerr << "Unable to get model[" << name << "] param key\n";
          if (!this->simIface->GetEntityParamValue(name, i, paramValue))
            std::cerr << "Unable to get model[" << name << "] param value\n";

          std::cout << paramKey << ": " << paramValue << "\n";
        }
        std::cout << "\n";
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Remove a model from the world
  public: void Kill()
  {
    if (this->params.size() < 2)
      std::cerr << "Missing model name\n";
    else
    {
      // Kill all the passed in models
      for (unsigned int i=1; i < this->params.size(); i++)
      {
        this->factoryIface->DeleteEntity( params[i] );

        while (strcmp((const char*)this->factoryIface->data->deleteEntity,"") != 0)
          usleep(10000);
      }
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  // Spawn a new model into the world
  public: void Spawn()
  {
    std::map<std::string, std::string>::iterator iter;

    if (this->params.size() < 2)
      std::cerr << "Missing model filename\n";
    else
    {
      FILE *file = fopen(params[1].c_str(),"r");
      if (file)
      {
        std::ostringstream stream;
        while (!feof(file))
        {
          char buffer[256];
          if (fgets(buffer, 256, file) == NULL)
            break;

          if (feof(file))
            break;
          stream << buffer;
        }
        std::string model = stream.str();

        for (iter = this->yamlValues.begin(); 
             iter != this->yamlValues.end(); iter++)
        { 
          boost::regex e1(std::string("(<model.*?") + iter->first + "\\h*?=\\h*?)\".*?\"(.*?<body)"); 
          boost::regex e2(std::string("(<model.*?<") + iter->first + ")>.*?</(" + iter->first + ">.*?<body)"); 

          std::string fmt1 = std::string("$1\"") + iter->second + "\"$2";
          std::string fmt2 = std::string("$1>") + iter->second + "</$2";

          model = boost::regex_replace(model,e1, fmt1, boost::format_first_only);
          model = boost::regex_replace(model,e2, fmt2, boost::format_first_only);
        }

        strcpy((char*)this->factoryIface->data->newModel, model.c_str());
      }
      else
        std::cerr << "Unable to open file[" << this->params[1] << "]\n";

      fclose(file);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  public: void Set()
  {
    if (this->params.size() < 2)
      std::cerr << "Missing model name\n";
    else
    {
      std::string name = this->params[1];

      std::map<std::string, std::string>::iterator iter;
      for (iter = this->yamlValues.begin(); 
           iter != this->yamlValues.end(); iter++)
      {
        if (iter->first == "linear_vel" || iter->first == "linear_accel" ||
            iter->first == "angular_vel" || iter->first == "angular_accel")
        {
          std::vector<std::string> strs;
          boost::split(strs, iter->second, boost::is_any_of("\t "));
          libgazebo::Vec3 vec( boost::lexical_cast<float>(strs[0]),
                            boost::lexical_cast<float>(strs[1]),
                            boost::lexical_cast<float>(strs[2]));

          if (iter->first == "linear_vel")
            this->simIface->SetLinearVel(name, vec);
          else if (iter->first == "linear_accel")
            this->simIface->SetLinearAccel(name, vec);
          else if (iter->first == "angular_vel")
            this->simIface->SetAngularVel(name, vec);
          else
            this->simIface->SetAngularAccel(name, vec);
        }
        else
          this->simIface->SetEntityParamValue(name, iter->first, iter->second );
      }
    }
  }

  public: bool Run()
  {
    if (!Toolbase::Run())
    {
      if (this->params[0] == "list")
        this->List();
      else if (params[0] == "info")
        this->Info();
      else if (params[0] == "kill")
        this->Kill();
      else if (params[0] == "spawn")
        this->Spawn();
      else if (params[0] == "set")
        this->Set();
      else
        std::cerr << "Unknown command[" << this->params[0] << "]\n";
    }

    return true;
  }


  //////////////////////////////////////////////////////////////////////////////
  // Print out help information
  public: void Help()
  {
    std::cout << "gazebomodel is a command-line tool for printing out information about models in a gazebo world.\n";
    std::cout << "\n";
    std::cout << "Usage: gazebomodel <command> <option_1> ... <option_n>\n";
    std::cout << "\n";

    std::cout << "Commands:\n";

    this->PrintCommands("gazebomodel");

    std::cout << "\tgazebomodel list \t List all the models\n";
    std::cout << "\tgazebomodel info \t Show info about a model(s)\n";
    std::cout << "\tgazebomodel kill \t Remove a model(s) from the world\n";
    std::cout << "\tgazebomodel spawn \t Insert a model into the world\n";
    std::cout << "\tgazebomodel set \t Get a parameter's value\n";
  }
};

////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ModelTool tool;
  tool.Init(argc, argv);
  tool.Run();

  return 1;
}
