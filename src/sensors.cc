#include "Sensors.hh"

int main(int argc, char **argv)
{
  gazebo::Sensors *sensor = new gazebo::Sensors();

  delete sensor;
  return 1;
}
