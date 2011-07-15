#include <boost/lexical_cast.hpp>
#include "transport/Transport.hh"
#include "Master.hh"
#include "gazebo_config.h"

gazebo::Master *master = NULL;
bool stop = false;

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
  master->Stop();
  stop=true;
  return;
}

int main(int /*argc*/, char ** /*argv*/)
{
  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  std::string host = "";
  unsigned short port = 0;

  if ( !gazebo::transport::get_master_uri(host,port) )

  master = new gazebo::Master();
  master->Init(port);
  master->Run();

  // wait loop
  while (!stop)
  {
    usleep(1000000);
  }

  master->Fini();

  delete master;
  master = NULL;

  return 1;
}
