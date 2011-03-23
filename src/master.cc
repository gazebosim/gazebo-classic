#include "Master.hh"


int main(int argc, char **argv)
{
  gazebo::Master *master = new gazebo::Master();
  master->Init(11345);
  master->Run();

  return 1;
}
