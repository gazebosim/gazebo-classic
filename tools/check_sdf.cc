
#include "sdf/parser/parser.hh"

std::vector<std::string> params;

using namespace sdf;

void help()
{
  std::cout << "gzsdf is a command-line tool for getting information about SDF files.\n\n";
  std::cout << "gzsdf <option> [sdf file]\n\n";
  std::cout << "Commands:\n";
  std::cout << "\tgzsdf describe\tPrint the SDF format\n";
  std::cout << "\tgzsdf check\tCheck the SDF format for the given file\n\n";
  std::cout << "\tgzsdf print\tPrints SDF, useful for debugging parser and as a conversion tool\n\n";
}

int main(int argc, char** argv)
{
  // Get parameters from command line
  for (int i=1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back( p );
  }

  boost::shared_ptr<SDF> sdf(new SDF());
  if (!init(sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return -1;
  }

  if (params.size() == 0 || params[0] == "help" || params[0] == "h")
  {
    help();
  }
  else if (params[0] == "check")
  {
    if (params.size() < 2)
    {
      std::cerr << "Error: Expecting an xml file to parse\n\n";
      help();
      return -1;
    }

    if (!readFile(params[1], sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return -1;
    }
    std::cout << "Check complete\n";
  }
  else if (params[0] == "describe")
  {
    sdf->PrintDescription();
  }
  else if (params[0] == "print")
  {
    if (params.size() < 2)
    {
      std::cerr << "Error: Expecting an xml file to parse\n\n";
      help();
      return -1;
    }

    if (!readFile(params[1], sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return -1;
    }
    sdf->PrintValues();
  }
  else
  {
    std::cerr << "Error: Unknown option[" << params[0] << "]\n";
    help();
  }

  return 0;
}


