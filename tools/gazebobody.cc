#include <iostream>
#include <stdio.h>
//#include <sys/select.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "Toolbase.hh"

class BodyTool : public Toolbase
{

  //////////////////////////////////////////////////////////////////////////////
  // Print out info for one body. Recurses through all child models
  public: void PrintBody(std::string name, std::string prefix)
  {
    std::string type;
    gazebo::Pose pose;

    this->simIface->GetEntityType(name, type);
    if (type == "body")
      std::cout <<  name << "\n";

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
        this->PrintBody(childName, prefix+"  ");
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Print out a list of all the bodies
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

      this->PrintBody(name, "");
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Remove a body from the world
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
  public: bool Run()
  {
    if (!Toolbase::Run())
    {
      if (this->params[0] == "list")
        this->List();
      else if (params[0] == "kill")
        this->Kill();
      else
        std::cerr << "Unknown command[" << this->params[0] << "]\n";
    }

    return true;
  }


  //////////////////////////////////////////////////////////////////////////////
  // Print out help information
  public: void Help()
  {
    std::cout << "gazebobody is a command-line tool for manipulating bodies in a gazebo world.\n";
    std::cout << "\n";
    std::cout << "Usage: gazebobody <command> <option_1> ... <option_n>\n";
    std::cout << "\n";

    std::cout << "Commands:\n";

    this->PrintCommands("gazebobody");

    std::cout << "\tgazebobody list \t List all the models\n";
    std::cout << "\tgazebobody info \t Show info about a model(s)\n";
    std::cout << "\tgazebobody kill \t Remove a model(s) from the world\n";
    std::cout << "\tgazebobody spawn \t Insert a model into the world\n";
    std::cout << "\tgazebobody set \t Get a parameter's value\n";
  }
};


////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  BodyTool tool;
  tool.Init(argc, argv);
  tool.Run();

  return 1;
}
