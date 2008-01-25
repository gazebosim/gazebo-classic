#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

////////////////////////////////////////////////////////////////////////////////
// Save an image frame
void SaveFrame(const char *filename, gazebo::CameraData *data)
{
  int i, width, height;
  FILE *file;

  file = fopen(filename, "w+");
  if (!file)
    return;

  width = data->width;
  height = data->height;

  int pixelSize = 3;
  int rowSize = width * pixelSize;

  // Write ppm  
  fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
  for (i = 0; i < height; i++)
    fwrite(data->image + i * rowSize, rowSize, 1, file);

  fclose(file);
}

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::CameraIface *camIface = new gazebo::CameraIface();

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

  /// Open the Camera interface
  try
  {
    camIface->Open(client, "camera_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the camera interface\n"
      << e << "\n";
    return -1;
  }

  int count = 0;
  char filename[50];
  while (true)
  {
    sprintf(filename,"save_%d.pnm",count);
    camIface->Lock(1);
    SaveFrame(filename, camIface->data);
    camIface->Unlock();

    usleep(100000);
    count++;
  }
  return 0;
}

