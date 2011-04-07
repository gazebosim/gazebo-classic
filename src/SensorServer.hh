#ifndef SENSORSERVER_HH
#define SENSORSERVER_HH

namespace gazebo
{
  class SensorServer
  {
    public: SensorServer();
    public: virtual ~SensorServer();

    public: void Load(const std::string &filename);
    public: void Run();
    public: void Quit();

    private: bool quit;
  };
}

#endif


