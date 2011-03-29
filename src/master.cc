#include "Master.hh"

gazebo::Master *master = NULL;

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int /*dummy*/ )
{
  master->Quit();
  return;
}

int main(int argc, char **argv)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  master = new gazebo::Master();
  master->Init(11345);
  master->Run();

  return 1;
}
