#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

gazebo::Client *client = NULL;
gazebo::SimulationIface *simIface = NULL;
gazebo::StereoCameraIface *stereoIface = NULL;
int saveCount = 0;

void SaveFrame()
{
  char tmp[1024];
  FILE *fp;
  
  sprintf(tmp, "frame%04d.pgm", saveCount);

  fp = fopen( tmp, "wb" );

  if (!fp)
  {
    printf( "unable to open file %s\n for writing", tmp );
    return;
  }

  fprintf( fp, "P6\n# Gazebo\n%d %d\n255\n", stereoIface->data->width, stereoIface->data->height);

  for (unsigned int i = 0; i<stereoIface->data->height; i++)
  {
    for (unsigned int j =0; j<stereoIface->data->width; j++)
    {
      unsigned char value = stereoIface->data->left_disparity[i*stereoIface->data->width+j] * 255;
      fwrite( &value, 1, 1, fp );
      fwrite( &value, 1, 1, fp );
      fwrite( &value, 1, 1, fp );
    }
  }

  fclose( fp );
  saveCount++;
}

int main()
{
  client = new gazebo::Client();
  simIface = new gazebo::SimulationIface();
  stereoIface = new gazebo::StereoCameraIface();


  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  /// Open the Stereo interface
  try
  {
    stereoIface->Open(client, "stereo_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }

  while (true)
  {
    stereoIface->Lock(1);
    SaveFrame();
    stereoIface->Unlock();

    usleep(100000);
  }

  return 0;
}


