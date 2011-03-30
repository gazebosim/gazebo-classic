#include <boost/lexical_cast.hpp>
#include "Master.hh"
#include "gazebo_config.h"

gazebo::Master *master = NULL;

////////////////////////////////////////////////////////////////////////////////
// Print the version/licence string
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int /*dummy*/ )
{
  master->Quit();
  return;
}

int main(int argc, char **argv)
{
  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  std::string master_uri = getenv("GAZEBO_MASTER_URI");
  int last_colon = master_uri.find_last_of(":") + 1;
  std::string port = master_uri.substr(last_colon, master_uri.size() - last_colon);

  master = new gazebo::Master();
  master->Init(boost::lexical_cast<int>(port));
  master->Run();

  return 1;
}
