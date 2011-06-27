
#include "sdf/parser/format_parser.hh"

using namespace sdf;
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }

  boost::shared_ptr<SDF> sdf(new SDF);
  if (!initFile(argv[1], sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return -1;
  }
  if (!readFile(argv[2],sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed\n";
    return -1;
  }

  sdf->PrintDescription();
  sdf->PrintValues();

  return 0;
}


